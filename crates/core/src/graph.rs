//! The plant graph: units as nodes, pipes as edges.
//!
//! Design rules:
//! - Pumps and valves are NODES with exactly one inlet and one outlet edge,
//!   so edges are uniform plain pipes and the solver sees one element kind
//!   per branch: pipe + (optional) node characteristic at its ends.
//! - petgraph is an implementation detail; it must not leak through pub APIs.
//! - Damage is graph surgery: a leak adds an edge to an Atmosphere node,
//!   a fire adds a heat source term to a node. No special-cased physics.

use crate::components::{Composition, Slate};
use crate::error::SimError;
use crate::stream::Stream;
use crate::units::*;
use petgraph::stable_graph::{EdgeIndex, NodeIndex, StableDiGraph};
use serde::{Deserialize, Serialize};

/// Stable, serializable node handle (index into the petgraph storage).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct NodeId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct EdgeId(pub u32);

impl From<NodeId> for NodeIndex {
    fn from(id: NodeId) -> Self {
        NodeIndex::new(id.0 as usize)
    }
}
impl From<EdgeId> for EdgeIndex {
    fn from(id: EdgeId) -> Self {
        EdgeIndex::new(id.0 as usize)
    }
}

// ---------------------------------------------------------------------------
// Nodes (units)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    /// Scenario-given name, unique per plant; frontends key on this.
    pub name: String,
    pub kind: NodeKind,
    /// External heat input [W] (fires, heaters). Damage model hooks in here.
    pub heat_input: Watt,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum NodeKind {
    /// Infinite feed at fixed pressure/temperature/composition.
    Source {
        pressure: Pascal,
        temperature: Kelvin,
        composition: Composition,
    },
    /// Infinite sink at fixed pressure. `temperature` and `composition` are the
    /// fluid it returns if the network ever drives flow *backwards* into it
    /// (network pressure below the sink's): an infinite reservoir has to have
    /// both to back-feed, and leaving either implicit would make reverse flow
    /// ill-defined.
    ///
    /// `composition` mirrors `temperature` exactly, and for the same reason —
    /// the alternative considered was a stateful sink that remembers what last
    /// flowed into it, which makes the back-fed fluid depend on tick history
    /// rather than on the plant definition. Reverse flow into a sink is not
    /// hypothetical: `energy_invariants.rs`'s chain proptest already generates
    /// it.
    Sink {
        pressure: Pascal,
        temperature: Kelvin,
        composition: Composition,
    },
    /// The outside world; leak edges terminate here. Fixed at P_ATM and,
    /// symmetrically, at T_AMBIENT.
    Atmosphere,
    /// Vertical cylindrical tank, vented (gas blanket pressure = P_ATM for
    /// M1; pressurized vessels are a later fidelity step).
    Tank(TankState),
    /// Capacitive gas vessel: a holdup whose STATE IS PRESSURE, not level
    /// (docs/DESIGN.md §3a fork 2). Knock-out drum, receiver, blowdown vessel.
    ///
    /// The one node kind that is neither pinned nor zero-volume. Its mass balance
    /// carries an accumulation term,
    ///
    /// ```text
    /// Σ_e ṁ_e(P) − C·(P − Pⁿ)/dt = 0,   C = V·M̄/(R·T)  [kg/Pa]
    /// ```
    ///
    /// so one hydraulic solve stops being a steady state and becomes one
    /// implicit-Euler step of a DAE. `C` is EXACT rather than a linearisation:
    /// `m(P) = P·V·M̄/(R·T)` is linear in `P` at fixed `T` and `M̄`.
    ///
    /// **Why not a `Tank` with a gas pressure law**, which would need no solver
    /// machinery at all: the explicit scheme is stable only while `dt·g/C < 2`,
    /// and liquid and gas capacitance are five orders apart. `tank_pump_valve`'s
    /// supply tank sits at 8.2e-7, six orders inside; a 1 m³ drum on a 20 kg/s
    /// line at 0.2 bar sits at **4.2**, outside, and oscillates. That is ordinary
    /// plant, so a guard would refuse exactly the scenario — a small vessel
    /// relieving quickly — that M5 exists to simulate.
    ///
    /// **Capacitance is an ANCHOR.** A vessel needs no conducting path to a pinned
    /// node, because its own equation determines its pressure, so a closed gas
    /// system with no fixed node at all is well posed (`network::anchored_set`).
    ///
    /// GAS ONLY, enforced at load. `C = V·M̄/(R·T)` is the ideal-gas relation;
    /// a liquid holdup is incompressible and its capacitance is not this number.
    /// The mirror of the tank's liquid-only guard, and it costs nothing: the two
    /// kinds partition the holdups by phase.
    ///
    /// No `ambient_ua`, deliberately. Nothing in M5.3 reads one, and an
    /// authoritative-looking field no code consumes is how an author comes to
    /// believe the model uses something it does not — the argument that keeps a
    /// "gas Cv" out of M5.4. A fire still reaches it through `Node::heat_input`.
    Vessel(VesselState),
    /// Centrifugal pump: head curve H(Q) = h0 - a·Q² (Q in m³/s, H in m).
    Pump { h0: Meter, a: f64, on: bool },
    /// Control valve, ISA-style: Q = Cv_eff(opening)·sqrt(dP/SG).
    /// `cv_max` in SI-consistent form (m³/s at 1 Pa dP for SG=1) — the
    /// scenario loader converts from customary Cv units.
    ///
    /// `x_t` is the pressure differential ratio factor of IEC 60534-2-1's gas
    /// sizing equation: the choke sits at `x = F_k·x_T`, with `F_k = γ/1.40`
    /// DERIVED from the slate. Present exactly when the valve is in GAS service
    /// and absent otherwise — the loader enforces both directions off M5.2's
    /// topological single-phase analysis, so no second notion of "gas service"
    /// exists to disagree with it (docs/DESIGN.md §3a forks 4 and 6).
    ///
    /// It is the one genuinely new coefficient in M5.4 and it has NO DEFAULT, for
    /// the reason that defers pump `η`: a silent default is an invented value in
    /// disguise, and both the sizing gate and the choked-plateau gate would then
    /// pass for whatever was chosen. `Cv` is deliberately NOT duplicated — the
    /// standard uses one coefficient for both services.
    Valve {
        cv_max: f64,
        opening: f64,
        /// Absent for a liquid valve, so an all-liquid plant serializes exactly
        /// as it did before M5.4 and the regression anchor is untouched.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        x_t: Option<f64>,
    },
    /// Spring-loaded pressure safety valve: a pressure-actuated AREA, not a
    /// controller (docs/DESIGN.md §3a fork 5).
    ///
    /// Its opening is a smooth, **memoryless** function of its own upstream
    /// pressure — shut at or below `set_pressure`, ramping to full over the
    /// accumulation band above it — evaluated inside the solve alongside every
    /// other branch characteristic. No state, no tuning constants, no tick
    /// history. That is a deliberate scope boundary (a PSV is one short step from
    /// a controls subsystem, and M5 does not take it) *and* the physically honest
    /// model at this fidelity.
    ///
    /// A separate kind rather than a flag on `Valve`, on the `Cooler`-versus-
    /// negative-`Furnace` precedent: the intent belongs in the name, not in the
    /// presence of a field. It hydraulically IS a valve — same `cv_max`, same
    /// `x_t`, same ISA gas law, same fold-at-source — so it shares every code path
    /// a valve takes and differs only in where `opening` comes from.
    ///
    /// GIVEN UP, and stated rather than discovered: no blowdown hysteresis (a real
    /// PSV recloses below its set pressure), no chatter, and — inherited from the
    /// gas valve's symmetry — it passes REVERSE flow, which a real one does not.
    /// All three need element state, and state is what turns an element into a
    /// controller.
    ReliefValve {
        cv_max: f64,
        /// Set pressure [Pa] ABSOLUTE: at or below it the valve is shut.
        set_pressure: Pascal,
        /// Accumulation band [Pa] above the set pressure over which the opening
        /// ramps from 0 to 1. Full lift is at `set_pressure + accumulation`.
        accumulation: Pascal,
        /// As `Valve::x_t` — required in gas service, refused in liquid.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        x_t: Option<f64>,
    },
    /// Zero-volume mixing point.
    Junction,
    /// Fired heater: a duty delivered into the stream passing through it.
    ///
    /// Zero-volume like a pump or valve — a furnace's tube inventory is
    /// negligible against its duty, so its outlet temperature is algebraic
    /// (`T_out = T_in + Q/(ṁ·cp)`) rather than a state. Hydraulically it is a
    /// plain pass-through at M2: the tube-side pressure drop belongs to the
    /// connecting pipes' resistance, not to a device characteristic.
    ///
    /// `duty` is the heat actually delivered to the process fluid [W], not a
    /// firing rate — combustion efficiency is a later fidelity step. Duty 0 is
    /// an unlit furnace; there is no separate `on` flag because there is
    /// nothing for one to express that 0 does not.
    ///
    /// Deliberately NOT stored in `Node::heat_input`: that field is the damage
    /// model's hook (fires), and a fire on a furnace must ADD to its duty, not
    /// overwrite the operator's setpoint. See `energy::heat_load`.
    Furnace { duty: Watt },
    /// Cooler: a duty *removed* from the stream passing through it.
    ///
    /// Structurally the furnace's mirror — zero-volume, hydraulically a
    /// pass-through, algebraic outlet temperature — and `duty` is likewise a
    /// non-negative magnitude: `energy::heat_load` applies the sign, SUBTRACTING
    /// a cooler's duty where it adds a furnace's.
    ///
    /// A separate unit rather than a negative-duty `Furnace`, deliberately. A
    /// bare signed number in a scenario file cannot be read without knowing
    /// which sign convention its unit uses, and a sign typo would silently turn
    /// a heater into a chiller. With two units the intent is in the name, and
    /// negative duty becomes meaningless input that both the loader and
    /// `Command::SetFurnaceDuty`/`SetCoolerDuty` reject.
    ///
    /// A fire (`Node::heat_input`) on a cooler correctly *fights* the cooling
    /// rather than replacing it, for free — `heat_load` sums the two terms.
    ///
    /// KNOWN LIMITATION: a fixed duty has no coolant-temperature floor, so a
    /// large duty on a small flow cools past the coolant, past ambient, and in
    /// the limit past 0 K. Only the last of those is detectable without a
    /// coolant model, and `mix_inflows` rejects it. Cooling to a realistic
    /// approach temperature is the `HeatExchanger`'s job (M2.2), not this one's.
    Cooler { duty: Watt },
    /// One side of a two-stream heat exchanger.
    ///
    /// A side is an ordinary zero-volume pass-through — hydraulically identical
    /// to a junction — and carries NO parameters of its own. What makes it an
    /// exchanger is the `HeatExchangerCoupling` naming it and its partner; this
    /// variant only says "I am a side", which is what `energy::is_zero_volume`
    /// and `energy::boundary_temperature` need to recognize.
    ///
    /// The effectiveness deliberately lives on the coupling rather than here.
    /// It is a property of the PAIR, and storing it once makes a pair whose two
    /// halves disagree about ε unrepresentable — the same instinct that made
    /// `Furnace` and `Cooler` separate units instead of one signed duty: put the
    /// invariant in the type, not in a convention.
    ///
    /// Neither side is "the hot one". Which way heat flows is decided per tick
    /// by the sign of `T_a_in − T_b_in`, so an exchanger whose duty reverses
    /// (seasonal service, a startup transient) needs no reconfiguration.
    HeatExchanger,
    /// Fixed cut-point distillation column (simple fidelity): one feed in, N
    /// draws out, split by the feed's boiling-range, NOT by the draws' hydraulic
    /// resistances (docs/DESIGN.md §5).
    ///
    /// The unit that does not fit the M2 mould. Every earlier device is a
    /// hydraulic pass-through the flow solver never learns is special; a column
    /// is one-in-N-out and the split comes from *composition*. Reconciling that
    /// with per-component mass conservation forced two structural choices, argued
    /// in the DESIGN note before any code:
    ///
    /// - **Fixed-pressure and zero-volume.** `pressure` is pinned (real columns
    ///   run on pressure control, and the overhead pressure is the setpoint that
    ///   sets the cut structure), so the feed edge is an ordinary pressure-driven
    ///   edge acting on it — a throttled feed valve or a draining supply lowers
    ///   `ṁ_feed`, which a free column with draws prescribed from the *previous*
    ///   feed could not do (it freezes the feed forever; see the note). The draws
    ///   are then `ṁ_drawᵢ = splitᵢ · ṁ_feed_now` with `Σ splitᵢ = 1`, so the
    ///   column is mass-neutral identically every tick — no holdup, no lag.
    /// - **Separation acts on the FEED, never on a holdup.** A holdup mixes to a
    ///   single composition and its outlets carry *that*, separating nothing. So
    ///   the column stores no inventory; `energy::column_separation` splits the
    ///   feed composition resolved this tick.
    ///
    /// The draw flows cannot be finalized in the hydraulic solve: the split needs
    /// the feed composition, which only exists after the transport sweep, and
    /// flow-split and composition-split must come from the SAME feed composition
    /// or per-component mass fails to balance at the column. So `Engine::tick`
    /// computes them post-sweep, and `network::edge_flows` only GUARDS the draw
    /// edges (reports zero rather than a bogus pressure-driven number). See the
    /// DESIGN note and `energy::column_separation`.
    ///
    /// Stated limitations (deliberate, not omissions): draws leave at the feed
    /// temperature; reverse feed flow is refused; a draw is insensitive to
    /// downstream back-pressure (a full product tank does not throttle it).
    Column {
        /// Operating pressure [Pa], pinned like a Source/Sink/Tank. Real columns
        /// run on pressure control; this is the operator setpoint.
        pressure: Pascal,
        /// Ramp width across each cut point [K]. A component boiling within
        /// `smearing` of a boundary lands partly in each adjacent draw. `0` is a
        /// sharp splitter — the degenerate case, kept representable.
        smearing: Kelvin,
        /// Draws in ascending boiling-point order (lightest first). Each feeds one
        /// outlet node and owns the band below its `upper_cut` and above the
        /// previous draw's. See `ColumnDraw`.
        draws: Vec<ColumnDraw>,
        /// Equilibrium-stage equipment, present iff `[fidelity] separation =
        /// "cascade"` (M7.3). `None` is the cut-point splitter, which needs no
        /// equipment at all — a boiling-range split is a property of the feed.
        ///
        /// The **declared-iff-used** correspondence this workspace already applies
        /// to `PseudoComponent::density` (required iff liquid) and a valve's `x_T`
        /// (required iff gas service): the loader requires this iff the cascade is
        /// selected and refuses it otherwise, so neither fidelity can carry a
        /// field the other silently ignores (DESIGN §5, fork 2).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        cascade: Option<CascadeSpec>,
    },
    /// Isothermal conversion reactor (simple fidelity): one feed in, one product
    /// out, held at a fixed reactor-outlet temperature `t_set`, whose chemistry
    /// comes from the engine's `ReactionModel` (DESIGN §5, "Simple reactor").
    ///
    /// Hydraulically it IS a furnace — zero-volume, 1-in-1-out, total-mass-neutral
    /// — so it reuses every furnace arm in the solver (`classify`,
    /// `fixed_pressure`, `validate_degrees`) unchanged. What is new is chemistry,
    /// and it collides with both M3 conservation invariants at once: the reaction
    /// conserves TOTAL mass but not per-component mass (vs I7), and it moves
    /// chemical energy the sensible datum does not track (vs I6). Both are settled
    /// by design, not code branches:
    ///
    /// - **Isothermal at a ROT setpoint, not adiabatic.** Holding `t_set` makes the
    ///   reaction extent a pure function of a KNOWN temperature — no inner solve —
    ///   and is the operator's real handle. The reactor imposes `t_set` on its
    ///   outlet exactly as a furnace imposes a duty; the heat that costs is an
    ///   EMERGENT diagnostic (`energy::reactor_duty`), not a stored `duty` field.
    ///   Adiabatic (coupling `dC/dτ` and `dT/dτ` into a fixed point) is deferred.
    /// - **Chemistry lives in the `ReactionModel`, keyed by the slate.** The
    ///   kinetic lumps ARE slate components, resolved by name; `react` maps the
    ///   feed composition to products at `t_set`. The single uniform outlet means
    ///   the reactor needs no per-draw composition machinery (unlike the column).
    ///
    /// `t_set` is a setpoint the vessel holds regardless of flow, so unlike a
    /// zero-volume mixing point the reactor's resolved temperature is `t_set`
    /// even with no inflow — there is nothing to mix, and the setpoint is a
    /// config fact, not a derived value.
    Reactor {
        /// Held reactor-outlet temperature [K] (the ROT setpoint). Imposed on the
        /// product stream; the heat to hold it is emergent, not configured.
        t_set: Kelvin,
        /// Residence time [s]. Unused by the lookup fidelity but passed to
        /// `ReactionModel::react`; the M4.2 kinetics integrate over it.
        tau: Seconds,
    },
}

/// One draw of a `Column`: the outlet it feeds and the top of its boiling-range
/// band.
///
/// The ordered `draws` list defines the cut points implicitly — draw `i`'s band
/// runs from draw `i−1`'s `upper_cut` (or −∞ for the lightest) up to its own.
/// `upper_cut = None` marks the heaviest draw, the open-topped catch-all that
/// every component above the last finite cut lands in; making it `None` rather
/// than `+∞` keeps the field JSON-serializable (snapshots carry `NodeKind`) and
/// makes "this is the residue draw" a type-level fact rather than a magic value.
/// The `stage` / `draw_ratio` pair is the CASCADE's way of locating and sizing a
/// draw, and it is the exact inverse of `upper_cut`: required by the cascade,
/// refused by the splitter, and vice versa. Same declared-iff-used
/// correspondence as `NodeKind::Column::cascade`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColumnDraw {
    /// The product node this draw feeds. Resolved from a name by the loader,
    /// exactly as a `HeatExchangerCoupling`'s sides are.
    pub outlet: NodeId,
    /// Upper boiling-point boundary of this draw's band [K]; `None` for the
    /// heaviest (open-topped) draw — and `None` for **every** draw under the
    /// cascade fidelity, which locates a draw by stage instead.
    pub upper_cut: Option<Kelvin>,
    /// Which equilibrium stage this draw leaves from (cascade fidelity only).
    ///
    /// `0` is the **total condenser** — the distillate — which is not an
    /// equilibrium stage; `1..=N` are the stages, and `N` is the **reboiler**,
    /// which is (note correction 3, and the convention the Fenske exponent rests
    /// on). Every stage in between is a liquid side draw.
    ///
    /// A plain `u32` rather than a `Condenser | Stage(n) | Reboiler` sum type on
    /// purpose: the integer *is* the position, `0` and `N` are already
    /// distinguished by the stage count, and adding a variant would change
    /// `ColumnDraw`'s serialized shape for every existing golden snapshot
    /// (snapshots carry `NodeKind`) while moving no number.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stage: Option<u32>,
    /// This draw's **mass** flow as a fraction of the column feed (cascade
    /// fidelity only): `D/F` for the distillate, `S_i/F` for a side draw.
    ///
    /// Mass, not molar, and that is load-bearing rather than a units convention
    /// (DESIGN §5, correction 1): a molar `D/F` would make the mass split depend
    /// on the distillate composition still being solved for, so total mass would
    /// close only *at convergence* instead of exactly. `None` on the heaviest
    /// (last) draw — the bottoms is `1 − Σ others` by subtraction and is never
    /// specified, which is what makes `Σ splitᵢ = 1` an identity of the
    /// specification.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub draw_ratio: Option<f64>,
}

impl ColumnDraw {
    /// A draw located by boiling range — the cut-point splitter's shape.
    /// `upper_cut = None` is the heaviest, open-topped draw.
    pub fn by_cut(outlet: NodeId, upper_cut: Option<Kelvin>) -> Self {
        Self {
            outlet,
            upper_cut,
            stage: None,
            draw_ratio: None,
        }
    }

    /// A draw located by stage and sized by a mass ratio — the cascade's shape.
    /// `draw_ratio = None` is the bottoms, whose share is `1 − Σ others`.
    ///
    /// Two constructors rather than one so the two fidelities' fields cannot be
    /// mixed by accident at a call site; the loader enforces the same exclusion
    /// on a file (DESIGN §5, fork 2).
    pub fn by_stage(outlet: NodeId, stage: u32, draw_ratio: Option<f64>) -> Self {
        Self {
            outlet,
            upper_cut: None,
            stage: Some(stage),
            draw_ratio,
        }
    }
}

/// A cascade column's equipment: how many equilibrium stages, where the feed
/// enters, and how hard it is refluxed (M7.3, DESIGN §5 forks 2 and 3).
///
/// What is deliberately NOT here: any absolute flow. Fork 3's verdict is that a
/// specification containing `D = 3.0 kg/s` re-runs the failure that killed M3.2's
/// prescribed-draw column — it either freezes the feed or creates mass in a
/// zero-volume node, while converging and conserving. The distillate and side
/// draw *ratios* live on `ColumnDraw::draw_ratio`; the total through the column
/// stays hydraulically determined.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CascadeSpec {
    /// Number of equilibrium stages `N`, **counting the reboiler and excluding
    /// the total condenser**.
    ///
    /// Stated here because a gate turns on it: Fenske at total reflux is
    /// `(x_D/(1−x_D))·((1−x_B)/x_B) = α^N`, and the textbook `N_min` counts the
    /// reboiler as a stage and does not count a total condenser. A convention
    /// left implicit makes an "exact, derivable" gate pass or fail on an
    /// off-by-one (DESIGN §5, correction 3).
    pub stages: u32,
    /// Which stage the feed enters, in `1..=stages`. A **saturated liquid** feed
    /// only in M7.3: under constant molar overflow the feed quality sets the
    /// internal liquid flow (`L' = L + q·F`), so a partly-vaporized feed changes
    /// the cascade rather than just an enthalpy term (correction 5).
    pub feed_stage: u32,
    /// Reflux ratio `R = L/D`, **molar** and **internal** — it never crosses the
    /// `SeparationModel` boundary as a flow, and it is one of the two real
    /// control-room handles this fidelity exposes (the other is `D/F`).
    ///
    /// `0` is legal and is not a degenerate plant: it is a column run with no
    /// reflux, and at `stages = 1` it is exactly the M7.2 single-stage flash,
    /// which is one of the cascade's gates.
    pub reflux_ratio: f64,
}

/// The thermal pairing of two `HeatExchanger` sides.
///
/// Hydraulically the two sides are unrelated: the flow solver never sees this
/// list, and the streams do not mix. The coupling exists only so the energy
/// sweep knows the pair must be resolved TOGETHER — each side's outlet depends
/// on the other side's inlet, which is not one of its own inflow edges.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeatExchangerCoupling {
    pub side_a: NodeId,
    pub side_b: NodeId,
    /// Effectiveness ε ∈ (0, 1]: the fraction of the thermodynamically maximum
    /// duty `C_min·(T_a_in − T_b_in)` this exchanger actually transfers.
    ///
    /// ε > 1 transfers more heat than the temperature difference makes
    /// available and would cross the outlet temperatures — a second-law
    /// violation — so it is rejected at every entry point. ε = 0 is a nonsense
    /// exchanger (use a plain pipe) and is likewise refused.
    pub effectiveness: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TankState {
    pub area: SquareMeter,
    pub height: Meter,
    pub mass: Kg,
    pub temperature: Kelvin,
    pub composition: Composition,
    /// Ambient heat transfer coefficient × exposed area, `UA` [W/K].
    ///
    /// Drives `Q = UA·(T_AMBIENT − T_tank)` — a SIGNED term, applied by
    /// `energy::ambient_exchange`, which heats a tank colder than ambient and
    /// cools one hotter with no second code path. See `energy::heat_load`.
    ///
    /// Defaults to ZERO: a perfectly insulated tank. That default is load
    /// bearing, not a placeholder — every scenario written before this field
    /// existed stays bit-identical, and `isothermal_plant.rs` keeps testing what
    /// it always tested. A tank that silently started leaking heat the day the
    /// field landed would turn that flat line into a lie.
    #[serde(default = "no_ambient_exchange")]
    pub ambient_ua: WattPerKelvin,
}

/// The `ambient_ua` default: a perfectly insulated body.
///
/// A local function rather than a blanket `Default` on the unit newtypes: `0`
/// is the physically meaningful "no exchange" here, whereas a default `Kelvin`
/// of 0 K would be a silent absurdity waiting for the first struct that forgot
/// to set one.
fn no_ambient_exchange() -> WattPerKelvin {
    WattPerKelvin::ZERO
}

/// A capacitive gas vessel's inventory. See `NodeKind::Vessel`.
///
/// Structurally the `TankState` of the gas world — mass, temperature and
/// composition integrate the same way — with `volume` in place of `area`/`height`
/// and no level anywhere, because a gas fills its container.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VesselState {
    /// Internal volume [m³]. Fixed geometry, never a state.
    pub volume: CubicMeter,
    pub mass: Kg,
    pub temperature: Kelvin,
    pub composition: Composition,
}

impl VesselState {
    /// Capacitance `C = dm/dP = V·M̄/(R·T)` [kg/Pa] at the vessel's current state.
    ///
    /// The whole relation, not a local slope: `m(P)` is linear in `P` at fixed
    /// `T` and `M̄`, so the Jacobian entry this produces is exact even though the
    /// edge density coefficients around it are frozen.
    pub fn capacitance(&self, slate: &Slate) -> f64 {
        self.volume.value() * self.composition.mean_molar_mass(slate).value()
            / (R_GAS * self.temperature.value())
    }

    /// Start-of-tick pressure `Pⁿ = m·R·T/(V·M̄)` [Pa].
    ///
    /// Written as `m/C` rather than spelled out again, so the pressure the
    /// residual measures its accumulation FROM and the capacitance that scales it
    /// are the same relation by construction. Stating the gas law twice would let
    /// `C·(P − Pⁿ)` mean something other than `m(P) − mⁿ`, which is the one thing
    /// the accumulation term must not be free to do.
    pub fn pressure(&self, slate: &Slate) -> Pascal {
        Pascal(self.mass.value() / self.capacitance(slate))
    }
}

impl TankState {
    /// The tank's own liquid density [kg/m³] — its contents' ideal-mixing density.
    ///
    /// **One owner, and that is the point of the method.** Until M8.2 the only
    /// caller that needed a tank's density computed this expression inline
    /// (`network::fixed_pressure`), which was harmless while the solver's head
    /// was the only consumer of a level. A control loop reads the SAME level and
    /// acts on it, so two inline copies of "which density does a tank have" would
    /// be two definitions of where the liquid surface is — the shape M7.4's
    /// `column_draw_at` rule exists to prevent, one milestone later and on the
    /// pair a reader is least likely to check.
    pub fn density(&self, slate: &Slate) -> KgPerM3 {
        self.composition.mixture_density(slate)
    }

    /// Liquid level [m]: `h = m / (ρ·A)`.
    ///
    /// Takes the slate rather than a caller-supplied density so the level a
    /// controller measures and the level the hydrostatic head is built on cannot
    /// come from different densities. See `density`.
    pub fn level(&self, slate: &Slate) -> Meter {
        Meter((self.mass / self.density(slate)).value() / self.area.value())
    }

    /// The mass `m = ρ·A·h` [kg] of this tank's own liquid standing at `level`.
    ///
    /// **One owner, in the loader's association, and the association is the
    /// point** (M23, docs/DESIGN.md §27 fork 4). The loader computes a tank's
    /// initial inventory through this method, and `capacity` is this method at
    /// the brim, so a tank declared exactly full holds exactly its capacity —
    /// to the bit, by construction. A level comparison would not: a level
    /// declared at load reads one ULP high on some compositions (M22), and a
    /// tank declared full would then spill a rounding error forever.
    pub fn mass_at_level(&self, slate: &Slate, level: Meter) -> Kg {
        Kg(self.density(slate).value() * self.area.value() * level.value())
    }

    /// How much of THIS liquid the shell holds, `ρ(x)·A·H` [kg]. A tank whose
    /// contents are changing has a different capacity every tick, so the engine
    /// asks at the end-of-tick composition.
    pub fn capacity(&self, slate: &Slate) -> Kg {
        self.mass_at_level(slate, self.height)
    }

    /// Hydrostatic pressure at the tank bottom nozzle.
    /// P = P_atm + ρ·g·h (vented tank).
    pub fn bottom_pressure(&self, slate: &Slate) -> Pascal {
        let density = self.density(slate);
        Pascal(P_ATM.value() + density.value() * G * self.level(slate).value())
    }
}

// ---------------------------------------------------------------------------
// Control loops (M8)
// ---------------------------------------------------------------------------

/// Stable handle for one control loop: its index into `PlantGraph::controls`.
///
/// The `NodeId`/`EdgeId` shape, and for the same reason — the command surface
/// has to name one loop from outside the engine (`Command::SetSetpoint`), and a
/// name would make the frontend contract depend on a string the scenario author
/// chose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct LoopId(pub u32);

/// Whether a loop drives its actuator, or a human does.
///
/// `Manual` is not "the loop is deleted": the loop still takes its measurement
/// and still reports a faceplate, it simply does not write. That distinction is
/// what makes the loop-off counterfactual (docs/DESIGN.md §10 fork 6, gate 1) a
/// run of the SAME plant rather than of a different one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode {
    Auto,
    Manual,
}

/// Which plant quantity a loop regulates.
///
/// **Two variants as of M10, and the second one is the milestone**
/// (docs/DESIGN.md §12). M8.2 shipped one — a tank's level — because both halves
/// of it already existed and were already load-bearing, so that slice added no
/// measurement path and no actuator alongside the control machinery it was there
/// to test. M8 then *claimed* the seam was variable-agnostic, and the only way to
/// find out was to add a variable.
///
/// **A vessel's pressure needs no new measurement path either, and the note that
/// deferred it said otherwise.** §10 fork 3 split the world into stored and
/// solved quantities and put pressure on the solved side — "lives in
/// `last_solution` / `NodeStates`, both of which are empty before the first
/// tick". That is false for a `Vessel`, whose pressure is `m/C` with `m` on this
/// graph: stored, real from load, and exactly the declared figure because the
/// loader builds the initial mass as `P · capacitance` through the same method
/// `pressure` divides by. It is true of a `Junction`, and that is where the
/// deferral survives — scoped to the node kinds it is actually true of rather
/// than to the variable. See `PlantGraph::measure`.
///
/// **The third variant, temperature, landed in M17 on the same correction made a
/// fourth time** (docs/DESIGN.md §21). This doc used to say "a temperature really
/// is a `NodeStates` quantity" — false for both holdups, whose `temperature` is a
/// field on `TankState` and `VesselState`, real from load and written back every
/// tick. It is true of a zero-volume node's temperature (a furnace's or a
/// cooler's outlet, a junction's mix), and `measure` refuses exactly those.
///
/// **The fourth variant, flow, landed in M20** (docs/DESIGN.md §24). This doc
/// used to close "Flow stays deferred … a flow lives on an edge, which nothing in
/// `measure`'s signature can name" — true of the signature, and paid with
/// `MeasurementPoint`, so a loop now measures at a node OR a pipe. A pipe's flow
/// is on the graph (`Pipe::stream.mass_flow`) but the loader stores an
/// initialiser zero there, not a declaration, so it is ABSENT at load exactly as
/// a furnace outlet is, and `measure` reads it from the last hydraulic solution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MeasuredVariable {
    Level,
    Pressure,
    Temperature,
    Flow,
}

impl MeasuredVariable {
    /// The variable as a noun in a message: "has no flow yet".
    pub fn noun(self) -> &'static str {
        match self {
            MeasuredVariable::Level => "level",
            MeasuredVariable::Pressure => "pressure",
            MeasuredVariable::Temperature => "temperature",
            MeasuredVariable::Flow => "flow",
        }
    }

    /// The scenario key that carries this variable's setpoint, unit included.
    ///
    /// Used to phrase the loader's refusals — so a message can name the key a
    /// file SHOULD have written rather than merely the one it did — and, since
    /// M10, to name the OTHER variable's key when a file writes that one instead.
    ///
    /// **`setpoint_bar`, not the `setpoint_pa` `ControlDef` predicted**
    /// (docs/DESIGN.md §12 fork 3): every pressure a scenario declares is in bar,
    /// six keys across four node kinds, and a `_pa` here would be the only one
    /// that is not.
    pub fn setpoint_key(self) -> &'static str {
        match self {
            MeasuredVariable::Level => "setpoint_m",
            MeasuredVariable::Pressure => "setpoint_bar",
            MeasuredVariable::Temperature => "setpoint_c",
            // The first variable whose file unit IS its SI unit (docs/DESIGN.md
            // §24 fork 4): the engine publishes kg/s, and an operator's t/h is a
            // display conversion at the frontend (rule 4).
            MeasuredVariable::Flow => "setpoint_kg_per_s",
        }
    }

    /// The scenario key that carries this variable's proportional gain.
    ///
    /// A gain's unit is the setpoint's reciprocal, so this pairs with
    /// `setpoint_key` and is chosen by the same variable — `gain_per_m` on a
    /// level loop, `gain_per_bar` on a pressure loop.
    ///
    /// **The silent trap this key exists to make loud** (docs/DESIGN.md §12 fork
    /// 3): a controller's arithmetic is in SI, so a `gain_per_bar` must be
    /// divided by the same 1e5 the setpoint is multiplied by. Converting one and
    /// not the other is a factor of 100 000 that no type catches, because a gain
    /// is a bare `f64` all the way into `ProportionalController::new`. The loader
    /// does both conversions at one site for exactly that reason.
    pub fn gain_key(self) -> &'static str {
        match self {
            MeasuredVariable::Level => "gain_per_m",
            MeasuredVariable::Pressure => "gain_per_bar",
            // **Per KELVIN, and the setpoint's `_c` is not a typo beside it**
            // (docs/DESIGN.md §21 fork 5). A gain multiplies a temperature
            // DIFFERENCE, and a difference of 1 °C is 1 K, so this key converts by
            // nothing where `setpoint_c` converts by `+ 273.15` — the inverse of
            // the pressure pair above, where both sides convert. Copying that
            // pattern ("convert both at one site") would add the offset to the
            // gain. `_per_k` says there is no offset to apply; `smearing_k` is
            // the format's precedent.
            MeasuredVariable::Temperature => "gain_per_k",
            // Per kg/s, converted by nothing, and so is the setpoint: the one
            // variable with no conversion at all, so the trap the two keys above
            // exist to make loud cannot be written here (§24 fork 4).
            MeasuredVariable::Flow => "gain_per_kg_per_s",
        }
    }
}

/// Where a loop's measurement is taken: at a node, or on a pipe (M20,
/// docs/DESIGN.md §24 fork 1).
///
/// **An enum rather than a second field beside the node**, because with two
/// `Option`s "both" and "neither" become representable and every reader has to
/// decide which one wins. A level, a pressure and a temperature are read at a
/// `Node`; a flow on a `Pipe`. `PlantGraph::measure` and
/// `PlantGraph::check_setpoint` refuse every other pairing.
///
/// Measuring a valve NODE's throughput instead was rejected: no snapshot
/// publishes such a number, and a valve's two pipes differ by that node's solver
/// residual, so "the valve's flow" would have to pick one pipe silently anyway.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasurementPoint {
    Node(NodeId),
    Pipe(EdgeId),
}

/// What a loop writes: a piece of equipment, or another loop's setpoint (M25,
/// docs/DESIGN.md §29 fork 1).
///
/// **An enum for `MeasurementPoint`'s reason**: two `Option`s would make "both"
/// and "neither" representable. A `Node` is a valve's opening or a cooler's or
/// furnace's duty, as every loop before M25 wrote. A `Loop` is a CASCADE: this loop
/// is the primary, and its output is the named secondary's setpoint, as a fraction
/// of this loop's `setpoint_range`. The loader admits two levels only — a loop that
/// drives another may not itself be driven — which is also its cycle refusal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Actuator {
    Node(NodeId),
    Loop(LoopId),
}

impl Actuator {
    /// The node this actuator writes, or `None` for a cascade primary, which
    /// writes no equipment at all.
    pub fn node(self) -> Option<NodeId> {
        match self {
            Actuator::Node(node) => Some(node),
            Actuator::Loop(_) => None,
        }
    }

    /// The loop this actuator drives, or `None` for an equipment actuator.
    pub fn driven_loop(self) -> Option<LoopId> {
        match self {
            Actuator::Loop(id) => Some(id),
            Actuator::Node(_) => None,
        }
    }
}

/// A cascade primary's authority over its secondary's setpoint: the setpoints
/// its output `0` and `1` stand for (M25, docs/DESIGN.md §29 fork 2).
///
/// **In the SECONDARY's variable and unit**, because the numbers are its
/// setpoints. Both ends pass the secondary's own `PlantGraph::check_setpoint` at
/// load and `min < max` strictly, so every setpoint the primary can write is one
/// `Command::SetSetpoint` would also accept. It sits beside `max_duty` for §21 fork
/// 3's reason: it is the loop's statement of its own range, and nothing else reads
/// it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SetpointRange {
    pub min: ControlledValue,
    pub max: ControlledValue,
}

impl SetpointRange {
    /// Where `setpoint` sits in the range, as a fraction: `(sp − min)/(max − min)`.
    ///
    /// Inside `[0, 1]` for every setpoint inside the range, end points included,
    /// because the subtraction and the division both round monotonically and
    /// `max − min` is the largest numerator possible. A setpoint outside the range
    /// reads outside `[0, 1]`; the loader and `Command::SetSetpoint` refuse one.
    pub fn position(self, setpoint: ControlledValue) -> f64 {
        let min = self.min.magnitude();
        (setpoint.magnitude() - min) / (self.max.magnitude() - min)
    }

    /// The setpoint a position stands for: `min + u·(max − min)`, **clamped to
    /// the range**.
    ///
    /// The clamp is not cosmetic. At `u = 1` the sum need not round to `max`
    /// exactly, and a setpoint one ULP past it reads back through `position` as
    /// `1.0000000000000002` — a position `seed_from_output` refuses, on the tick a
    /// cascade opens or a primary goes to AUTO. Clamping to `[min, max]` keeps the
    /// round trip inside `[0, 1]` by `position`'s own argument.
    pub fn setpoint_at(self, position: f64) -> ControlledValue {
        let min = self.min.magnitude();
        let max = self.max.magnitude();
        self.min
            .with_magnitude((min + position * (max - min)).clamp(min, max))
    }

    /// Whether `setpoint` lies in `[min, max]`, end points included.
    pub fn contains(self, setpoint: ControlledValue) -> bool {
        setpoint.variable() == self.min.variable()
            && (self.min.magnitude()..=self.max.magnitude()).contains(&setpoint.magnitude())
    }
}

/// A regulated quantity — a setpoint or a measurement — carrying its own unit.
///
/// **The unit is in the type because it cannot be in the field name**
/// (docs/DESIGN.md §10 fork 4). Every quantity on `NodeSnapshot` says its unit in
/// its own name (`pressure_pa`, `temperature_k`, `heat_input_w`) because a
/// snapshot is plain serde data with no newtypes to carry one. A loop's setpoint
/// has no such name available: it is metres today and Pascals the moment pressure
/// control un-defers, so `setpoint_m` would be a lie on half the loops and a bare
/// `setpoint` would be a number whose unit depends on a SIBLING field. §7's own
/// answer — "unit-specific extras as tagged enums" — is this type.
///
/// Two consequences taken deliberately:
///
/// - The setpoint and the measurement are the **same type**, so a loop cannot
///   report a setpoint in one variable against a measurement in another. That is
///   M7.4's `column_draw_at` rule applied to the pair a reader is most likely to
///   subtract.
/// - Inside `core` the payload is a unit newtype, so rule 4 holds on the way in
///   as well as on the way out. `Meter` is `#[serde(transparent)]`, so the wire
///   form is still `{"variable":"level","m":6.0}` — a tagged number, not a
///   nested object.
///
/// **The second variant landed in M10, and it made two guards required that were
/// deliberately absent while there was one.** The note here used to read: "with
/// one variant, 'the setpoint's variable disagrees with the loop's' is
/// unrepresentable, and there is deliberately no guard for it — a refusal path
/// nothing can reach is a coverage claim that cannot be checked
/// (`a-counter-is-not-a-gate`); the moment a second variant lands, writing a
/// setpoint through `Command::SetSetpoint` becomes able to change what a loop
/// measures, and that refusal becomes required rather than optional." M10 is that
/// moment and pays it, in `Engine::apply`.
///
/// The one it did NOT name is `error` itself, below — see that method.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "variable", rename_all = "snake_case")]
pub enum ControlledValue {
    Level {
        m: Meter,
    },
    /// A vessel's absolute pressure.
    ///
    /// **The payload is Pascals where the scenario key is bar, and that
    /// asymmetry is deliberate** (docs/DESIGN.md §12 fork 3). `Pascal` is
    /// `#[serde(transparent)]` like `Meter`, so the wire form is
    /// `{"variable":"pressure","pa":1.2e6}` — a tagged number, the same shape the
    /// level variant has. The format's own convention is `pressure_bar` in and
    /// `pressure_pa` out; this variant follows the format rather than the level
    /// loop, and rule 4 holds on both sides because the unit is in the key going
    /// in and in the type coming out.
    Pressure {
        pa: Pascal,
    },
    /// A holdup's temperature (M17, docs/DESIGN.md §21).
    ///
    /// Kelvin out where the scenario key is °C in — the format's own
    /// `temperature_c` in, `temperature_k` out convention, as the pressure variant
    /// follows `pressure_bar` in, `pressure_pa` out. `Kelvin` is
    /// `#[serde(transparent)]`, so the wire form is
    /// `{"variable":"temperature","k":333.15}`.
    Temperature {
        k: Kelvin,
    },
    /// A pipe's mass flow, signed by the pipe's DECLARED direction (M20,
    /// docs/DESIGN.md §24).
    ///
    /// kg/s in and kg/s out — no conversion on either side, a first. `KgPerSec`
    /// is `#[serde(transparent)]`, so the wire form is
    /// `{"variable":"flow","kg_per_s":12.0}`. A negative value is a real
    /// measurement of flow running backwards through the pipe, and is published
    /// as one rather than clipped (`docs/DEFERRED.md` E11).
    Flow {
        kg_per_s: KgPerSec,
    },
}

impl ControlledValue {
    /// Which variable this value is of.
    ///
    /// `ControlLoop` stores no separate `variable` field: the setpoint IS the
    /// declaration of what the loop measures, so there is one owner and the two
    /// cannot drift apart. The loader still parses a `variable` key, because a
    /// file has to say which unit its setpoint key carries before the setpoint
    /// can be built — but it is consumed there and not stored twice.
    pub fn variable(self) -> MeasuredVariable {
        match self {
            ControlledValue::Level { .. } => MeasuredVariable::Level,
            ControlledValue::Pressure { .. } => MeasuredVariable::Pressure,
            ControlledValue::Temperature { .. } => MeasuredVariable::Temperature,
            ControlledValue::Flow { .. } => MeasuredVariable::Flow,
        }
    }

    /// The bare magnitude, in this variable's SI unit.
    ///
    /// The one place a unit is dropped, and it exists so a `Controller` impl can
    /// do arithmetic. Everything upstream of it — the measurement read, the
    /// setpoint, the command that writes one — is typed; a controller's gain
    /// carries the reciprocal unit implicitly, which is why the scenario key says
    /// so (`gain_per_m`).
    pub fn magnitude(self) -> f64 {
        match self {
            ControlledValue::Level { m } => m.value(),
            ControlledValue::Pressure { pa } => pa.value(),
            ControlledValue::Temperature { k } => k.value(),
            ControlledValue::Flow { kg_per_s } => kg_per_s.value(),
        }
    }

    /// The same variable with a different magnitude, in its SI unit — the inverse
    /// of `magnitude`, and the one place a bare number becomes a typed value again
    /// (a cascade primary's range map, M25).
    pub fn with_magnitude(self, value: f64) -> Self {
        match self {
            ControlledValue::Level { .. } => ControlledValue::Level { m: Meter(value) },
            ControlledValue::Pressure { .. } => ControlledValue::Pressure { pa: Pascal(value) },
            ControlledValue::Temperature { .. } => {
                ControlledValue::Temperature { k: Kelvin(value) }
            }
            ControlledValue::Flow { .. } => ControlledValue::Flow {
                kg_per_s: KgPerSec(value),
            },
        }
    }

    /// The loop's error, in this variable's SI unit: `measurement − setpoint` for
    /// a DIRECT-acting loop, `setpoint − measurement` for a REVERSE-acting one.
    ///
    /// **The error's sign convention lives here and nowhere else**, and from M18
    /// that includes the loop's direction of action (docs/DESIGN.md §22 fork 1).
    /// A positive error always RAISES the output. On a direct loop that means
    /// "above setpoint", so a drain opens on a high level and a cooler works
    /// harder on a hot tank. On a reverse loop it means "below setpoint", so a
    /// furnace fires harder on a cold tank. The action is an argument rather than
    /// a second function, and not a swap of the two arguments at a call site,
    /// because either of those would put the sign somewhere a reader cannot see
    /// it. A controller that computed its own
    /// difference would be free to disagree with the one a snapshot reader
    /// reconstructs from the two reported values, and the mutation this slice owes
    /// ("gain applied to the measurement instead of the error") only means
    /// anything while the error term is explicit and singly owned.
    ///
    /// **The sentence that used to close this doc became false in M10, and the
    /// method had to change with it.** It read: "both arguments are the same type
    /// by construction, so a level measurement cannot be differenced against a
    /// pressure setpoint." That was a property of there being ONE variant, not of
    /// the type — with two, `error(Pressure { pa: 5e5 }, Level { m: 4.0 })`
    /// subtracts metres from Pascals and returns a plausible `499996.0`. The type
    /// stops a level being differenced against a pressure only if something
    /// checks the variables match, and until here nothing did.
    ///
    /// So a mismatch returns `NaN` rather than a number. It does not propagate
    /// silently: `Engine::run_control_loops` already refuses a controller output
    /// that is not a finite fraction, naming the loop, so the existing rule-5
    /// backstop turns this into a diagnosed `SimError::Numerical` one call later.
    ///
    /// **This is a backstop and not the guard.** The engine only ever calls
    /// `measure(setpoint.variable())` against that same loop's setpoint, and
    /// `Command::SetSetpoint` refuses a value whose variable disagrees with the
    /// loop's — those two are what make a mismatch unreachable. The `NaN` is what
    /// happens if one of them is ever removed.
    ///
    /// The direct arm is the bare subtraction it always was, not `1.0 ×` it, so
    /// every loop written before M18 computes the same bits it did.
    pub fn error(measurement: Self, setpoint: Self, action: ControlAction) -> f64 {
        if measurement.variable() != setpoint.variable() {
            return f64::NAN;
        }
        match action {
            ControlAction::Direct => measurement.magnitude() - setpoint.magnitude(),
            ControlAction::Reverse => setpoint.magnitude() - measurement.magnitude(),
        }
    }
}

/// Which way a loop's output moves its measurement (M18, docs/DESIGN.md §22).
///
/// **DIRECT: raising the output LOWERS the measurement** — a drain on a level, a
/// vent on a pressure, a cooler on a temperature. **REVERSE: raising the output
/// RAISES it** — a furnace on a temperature, a valve on the flow in its own
/// pipe (M20). The loop DECLARES which, and the loader checks the declaration
/// against the actuator where the sign is physics (a cooler must be direct, a
/// furnace reverse, a valve holding its own flow reverse) and refuses reverse on
/// a level or pressure loop's valve, whose sign runs through a holdup and is
/// topology the loader does not check (`docs/DEFERRED.md` E8). A flow loop's
/// sign is checkable in one hop because a valve has exactly one inlet and one
/// outlet (docs/DESIGN.md §24 fork 3).
///
/// This is the fourth wording of the rule M8.4 first wrote as "a level loop must
/// actuate a drain", and the first that is not a special case of the direct
/// half. A negative gain stays refused: it would be a second way to say this.
///
/// `Direct` is the default because it is a TRUE statement about every loop
/// written before M18 — reverse action was refused when they were written — and
/// the snapshot skips the field when direct, so those loops publish the bytes
/// they always did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlAction {
    #[default]
    Direct,
    Reverse,
}

impl ControlAction {
    /// `true` for the default, so `ControlSnapshot` can skip it.
    pub fn is_direct(&self) -> bool {
        *self == ControlAction::Direct
    }
}

/// One regulating loop: what it measures, what it writes, and how.
///
/// **It lives beside the graph, not on it** (docs/DESIGN.md §10 fork 1). A
/// `NodeKind` was rejected because a controller conducts nothing and every graph
/// algorithm would have to skip it; a field on the actuator node was rejected
/// because the actuator field is exactly what the loop WRITES, and storing the
/// writer inside the written struct makes "who owns this opening" unanswerable
/// where `Engine::apply` has to answer it.
///
/// Not `Serialize`/`Deserialize`, unlike every other type in this file: the
/// algorithm is a boxed trait object selected at load, so a loop is not
/// round-trippable through the scenario format. What a frontend needs to see
/// travels as `snapshot::ControlSnapshot` instead, which is plain data.
#[derive(Debug)]
pub struct ControlLoop {
    /// Scenario-given name, unique per plant. What a faceplate is labelled with.
    pub name: String,
    /// Where the state is measured. A `Node` for a level loop (a `Tank`), a
    /// pressure loop (a `Vessel`), or a temperature loop (a `Tank` or `Vessel`,
    /// M17, or a `Furnace` or `Cooler` OUTLET, M19); a `Pipe` for a flow loop
    /// (M20, docs/DESIGN.md §24 fork 1). The loader refuses every other pairing
    /// through `PlantGraph::measure`, which is the single owner of which points
    /// can answer for which variable and carries a distinct reason for each
    /// refusal.
    pub measurement_point: MeasurementPoint,
    /// What this loop writes. A NODE: a `Valve` on a level, pressure or flow
    /// loop — on a flow loop, the valve whose own inlet or outlet pipe is
    /// measured; a `Cooler` (M17) or a `Furnace` (M18) on a temperature loop. The
    /// loader refuses every other pairing with its own reason (docs/DESIGN.md §21
    /// fork 3, §24 fork 3), and a `ReliefValve` always, since its opening is
    /// actuated by its own inlet pressure. Or a LOOP (M25, §29): this loop is a
    /// cascade primary and writes that loop's setpoint, through `setpoint_range`.
    pub actuator: Actuator,
    /// Which way the output moves the measurement: a furnace loop and a flow loop
    /// are `Reverse`, every other loop `Direct`. Passed into `ControlledValue::error` by every
    /// caller that reaches it — the load-time seed, the MANUAL→AUTO seed and the
    /// tick's update — so the loop is seeded and run against one sign
    /// (docs/DESIGN.md §22 fork 1).
    pub action: ControlAction,
    /// The loop's declared authority over a DUTY actuator: the cooler or furnace
    /// duty its full output `u = 1` stands for. `Some` exactly when the actuator
    /// is a `Cooler` or a `Furnace`, `None` on a valve, whose opening is already a
    /// fraction, and on a cascade primary, whose authority is `setpoint_range`.
    ///
    /// **On the loop, not on the cooler** (docs/DESIGN.md §21 fork 3).
    /// `NodeSnapshot::kind` serializes `NodeKind`, so a field on the cooler would
    /// move every published cooler plant's bytes — and it is the loop's statement
    /// of its own range, which no other reader of a cooler has any use for.
    /// Read only through `PlantGraph::actuator_position` and
    /// `PlantGraph::set_actuator_position`, the single owner of "this actuator's
    /// position as a fraction of its authority".
    pub max_duty: Option<Watt>,
    /// A cascade primary's declared authority over its secondary's setpoint (M25,
    /// docs/DESIGN.md §29 fork 2): `Some` exactly when `actuator` is a `Loop`.
    /// Read only through the same two accessors as `max_duty`.
    pub setpoint_range: Option<SetpointRange>,
    /// The target value, and — through `ControlledValue::variable` — the
    /// declaration of what this loop measures.
    pub setpoint: ControlledValue,
    pub mode: ControlMode,
    /// The control algorithm, boxed per loop.
    ///
    /// **The project's first `Vec<Box<dyn _>>` seam** (docs/DESIGN.md §10 fork 2).
    /// Every earlier seam — `FlowSolver`, `ThermoModel`, `ReactionModel`,
    /// `SeparationModel` — is an engine-wide singleton chosen by one string in
    /// `[fidelity]`, and a control algorithm is not that shape: one plant can want
    /// one loop type on a tank and another on a vessel, which `[fidelity]` has no
    /// way to say. Rule 2 is honoured and its ARITY is what changed.
    ///
    /// The genuinely new property, stated at the field because it is easy to miss:
    /// an impl of this trait **owns state** where every earlier seam's impls are
    /// pure. That is §3a fork 5's own definition of what turns an element into a
    /// controller, and it is why the box is part of the engine's inventory rather
    /// than part of its configuration. `ProportionalController` (M8.2) happens to
    /// be stateless; `PiController` (M8.3) is not.
    pub algorithm: Box<dyn crate::traits::Controller>,
    /// The measurement this loop last ACTED ON — not a re-read of what is true
    /// now.
    ///
    /// The two differ by one tick (fork 3), and reporting the fresh one would make
    /// a lagging loop look instantaneous, hiding the lag from exactly the person
    /// debugging it.
    ///
    /// **`None` exactly when the loop had nothing it could act on** (M19,
    /// docs/DESIGN.md §23): a furnace or cooler OUTLET before the first tick, or
    /// while it is stagnant, and a pipe's FLOW before the first tick (M20, §24 —
    /// a flow of zero is a measurement, so a flow is never absent after it).
    /// Never a stand-in and never "healthy" — M11's rule for
    /// `cavitation`. Every other measurement is a STORED quantity — a tank's mass,
    /// a vessel's mass, a holdup's temperature all live on the graph — so for
    /// those the loader seeds this by taking the measurement once and it is
    /// `Some` from load: a snapshot before the first tick reports a true level,
    /// pressure or temperature instead of a NaN or an absence. That is why the
    /// `Option` is byte-neutral for every loop written before M19, and also why
    /// no corpus run can defend the absent case (§23 fork 3).
    ///
    /// That contrast is sharpest on the pressure loop and is worth reading once:
    /// at tick 0 this field holds the vessel's declared pressure while
    /// `NodeSnapshot::pressure_pa` for the same node is NaN, because the solve has
    /// not run. Two different quantities that agree once the plant is running, and
    /// the fact that one exists before the other is the whole of what §12
    /// corrected. The temperature loop repeats it exactly: `temperature_k` is NaN
    /// at tick 0 and this holds the declared holdup temperature (§21, gate 1).
    pub last_measurement: Option<ControlledValue>,
    /// The actuator position this loop last put on the faceplate, dimensionless
    /// in `[0, 1]` — for a cooler, a fraction of `max_duty`.
    ///
    /// In `Auto` it is the controller's output, which is also what was written to
    /// the actuator. In `Manual` the loop writes nothing and this TRACKS the
    /// actuator's real position, which is what a DCS faceplate shows and what
    /// makes AUTO→MANUAL transfer free (fork 4). Seeded from the actuator's
    /// declared position at load.
    pub last_output: f64,
}

// ---------------------------------------------------------------------------
// Trips (M22)
// ---------------------------------------------------------------------------

/// Stable handle for one trip: its index into `PlantGraph::trips`.
///
/// `LoopId`'s shape and reason: `Command::ResetTrip` has to name one trip from
/// outside the engine, and a name would tie the frontend contract to a string
/// the scenario author chose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct TripId(pub u32);

/// Which side of its limit a trip guards (docs/DESIGN.md §26 fork 6).
///
/// Declared by the file with no default: an overfill trip and a low-level trip
/// on one tank differ only here, so a default would make the file's most
/// important word invisible.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TripDirection {
    /// Fires when the measurement is AT OR ABOVE the limit.
    High,
    /// Fires when the measurement is AT OR BELOW the limit.
    Low,
}

impl TripDirection {
    /// Whether `measurement` stands in this trip's condition against `limit`.
    ///
    /// **The single owner of the comparison, and firing and resetting both ask
    /// it** — a trip fires when this is `true` and may be reset only when it is
    /// `false`, so the reset test is the strict complement of the firing test by
    /// construction rather than by a second, hand-negated comparison that could
    /// disagree with it at the tie.
    ///
    /// **At the limit counts as reached** (`≥` for a high trip, `≤` for a low
    /// one). A trip setpoint is conventionally "reached", and the tie is the side
    /// a safety function takes. Which ties are exact on a loaded plant was
    /// measured rather than assumed (docs/DESIGN.md §26 fork 6): a vessel's
    /// pressure and a holdup's temperature are, a level is not always.
    ///
    /// # Errors
    /// `SimError::Numerical` if the two values are of different variables — the
    /// engine measures `limit.variable()` for exactly this call, so that is
    /// unreachable unless the two drift apart — or if the measurement is not
    /// finite. A NaN compares false both ways, which would read as "safe" on a
    /// high trip and a low one alike; a safety function does not get to say
    /// that about a number it could not compare (rule 5).
    pub fn reached(
        self,
        measurement: ControlledValue,
        limit: ControlledValue,
    ) -> Result<bool, SimError> {
        if measurement.variable() != limit.variable() {
            return Err(SimError::Numerical(format!(
                "a trip compared a {:?} measurement against a {:?} limit",
                measurement.variable(),
                limit.variable()
            )));
        }
        let (value, bound) = (measurement.magnitude(), limit.magnitude());
        if !value.is_finite() {
            return Err(SimError::Numerical(format!(
                "a trip's measurement is {value}, which cannot be compared with its limit"
            )));
        }
        Ok(match self {
            TripDirection::High => value >= bound,
            TripDirection::Low => value <= bound,
        })
    }
}

/// Whether a trip is watching or has fired (docs/DESIGN.md §26 fork 8).
///
/// **One enum rather than a `tripped: bool` beside an `Option<u64>`**, because
/// the pair could disagree — "tripped with no tick" and "armed at tick 400" —
/// and this makes both unrepresentable. `CascadeProfile`'s argument (M9.3b).
///
/// Tagged `status`, so the wire form inside `TripSnapshot::state` is
/// `{"status":"armed"}` or `{"status":"tripped","at_tick":1236}` rather than a
/// `state` key nested in a `state` field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum TripState {
    Armed,
    /// Latched. `at_tick` is the tick whose trip pass fired it — the number a
    /// snapshot of that tick carries — so a frontend sampling every tenth
    /// snapshot still knows exactly when. A reset returns the trip to `Armed`
    /// and the tick goes with it: this is "tripped now, since when", not "last
    /// tripped at" (a trip's history is `docs/DEFERRED.md` E15).
    Tripped {
        at_tick: u64,
    },
}

impl TripState {
    pub fn is_tripped(self) -> bool {
        matches!(self, TripState::Tripped { .. })
    }
}

/// One piece of equipment a trip drives to its safe state (docs/DESIGN.md §26
/// fork 3).
///
/// **A list of these, not one, and the reason is measured**: a stopped pump
/// keeps its resistance and conducts both ways, so on the reference plant
/// "stop the pump" alone sends 3 kg/s back from the higher tank. A trip that
/// must stop a flow names the pump AND a valve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TripAction {
    /// Stop a pump: its safe state is `on = false`.
    StopPump { pump: NodeId },
    /// Put a valve at a declared opening in `[0, 1]` — usually shut, but a vent
    /// or dump valve trips OPEN, so the file says which.
    SetValve { valve: NodeId, position: f64 },
}

impl TripAction {
    /// The node this action writes.
    pub fn equipment(self) -> NodeId {
        match self {
            TripAction::StopPump { pump } => pump,
            TripAction::SetValve { valve, .. } => valve,
        }
    }
}

/// One latching trip: what it watches, the limit, and what it does when the
/// limit is reached (docs/DESIGN.md §26).
///
/// **Beside the graph, like a loop, and a plain struct rather than a trait**
/// (fork 1). A P and a PI controller are two algorithms behind one call; a trip
/// has no second model. It is a comparison and a latch, and everything a real
/// safety system adds to it — a delay, voting, a bypass — is data on one trip
/// (`docs/DEFERRED.md` E15), not an alternative algorithm for it.
#[derive(Debug, Clone)]
pub struct Trip {
    /// Scenario-given name, unique per plant.
    pub name: String,
    /// Where the trip measures. Always a `Node` today: only a tank's level, a
    /// vessel's pressure and a holdup's temperature are admitted, because all
    /// three are stored and exist from load (fork 2). A trip on a quantity absent
    /// at load owes a rule for the missing measurement (`docs/DEFERRED.md` E13).
    pub measurement_point: MeasurementPoint,
    pub direction: TripDirection,
    /// The limit, and — through `ControlledValue::variable` — what is measured.
    pub limit: ControlledValue,
    /// Non-empty; each names one pump or valve and its safe state.
    pub actions: Vec<TripAction>,
    pub state: TripState,
    /// The measurement the last trip pass compared. `None` only before the
    /// first tick, when no pass has run.
    pub last_measurement: Option<ControlledValue>,
}

impl Trip {
    /// The safe opening this trip holds `valve` at, if it names that valve.
    pub fn valve_position(&self, valve: NodeId) -> Option<f64> {
        self.actions.iter().find_map(|a| match *a {
            TripAction::SetValve { valve: v, position } if v == valve => Some(position),
            _ => None,
        })
    }

    /// Whether this trip names `node` among its equipment.
    pub fn acts_on(&self, node: NodeId) -> bool {
        self.actions.iter().any(|a| a.equipment() == node)
    }
}

// ---------------------------------------------------------------------------
// Edges (pipes)
// ---------------------------------------------------------------------------

/// An edge's role in a declared leak path (docs/DESIGN.md §3b).
///
/// A leak is **not** a scalar sink inside a pipe's own equation — that shape was
/// rejected because `network::edge_flows` returns one `ṁ` per edge and every
/// mass balance, energy transport and dissipation term reads exactly that one
/// number. A leak is an EDGE to an `Atmosphere` node, created by the loader when
/// the scenario declares one, and inert until damaged.
///
/// The three states are one field rather than two `Option`s so that "this edge
/// is an orifice AND points at another orifice" is a state the type cannot
/// represent. Only the loader constructs the non-`None` variants; the pairing
/// they encode (upstream half ↔ its orifice) is what `Command::PuncturePipe`
/// routes through, so a hand-built graph that invents one is inventing a plant.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(tag = "role", rename_all = "snake_case")]
pub enum LeakRole {
    /// An ordinary pipe. Every edge in a scenario that declares no leak.
    #[default]
    None,
    /// The UPSTREAM half of a pipe the scenario declared punctureable, carrying
    /// the id of the orifice edge hanging off the junction between the halves.
    ///
    /// The upstream half keeps the declared pipe's NAME and is what
    /// `PuncturePipe { edge }` addresses, so the frontend contract still names
    /// the pipe the scenario author wrote. It is also where `leak_mass_flow` is
    /// reported — the end a frontend draws a spray from.
    Punctureable { orifice: EdgeId },
    /// The orifice edge itself: junction → `Atmosphere`, with the commanded
    /// area. `ZERO` is dormant — it compiles to `alpha = +∞`, which
    /// `network::compile_edge`'s existing `conducts` test already reads as
    /// "closed", so a dormant leak conducts nothing and needs no special case.
    Orifice { area: SquareMeter },
    /// A boil-off vent: tank → `Atmosphere`, built by the loader when
    /// `[fidelity] boiloff = "flash"` (M12, docs/DESIGN.md §14 fork 4).
    ///
    /// **Not a leak, and it shares this field anyway.** The question this field
    /// answers is not "is this edge damaged" but "is this edge an ordinary
    /// pressure-driven pipe, or a path to `Atmosphere` that something else
    /// owns" — a leak orifice is owned by `Command::PuncturePipe`, and a vent by
    /// `Engine::tick`'s holdup dynamics. One field keeps "this edge is an
    /// orifice AND a vent" unrepresentable, which is the same property the three
    /// variants above were collapsed into one enum for.
    ///
    /// **Its flow is PRESCRIBED, exactly like a column draw's.** A vent's rate
    /// comes from an enthalpy balance over the holdup, not from
    /// `ρ·branch.flow(dp)`, so `network::edge_flows` guards it to zero and
    /// `Engine::tick` writes the authoritative value afterwards. Leaving it
    /// pressure-driven would drain a tank to atmosphere through a pipe nobody
    /// declared — finite, deterministic, mass-conserving and wrong.
    ///
    /// **`emitter` is the holdup that OWNS this vent, and it is stored rather
    /// than derived (M14, docs/DESIGN.md §16 fork 3).** Through M13 every vent
    /// ended at an `Atmosphere`, so "the emitting end" could be read off the
    /// edge's direction — `build_boiloff_vents` writes tank → atmosphere and
    /// says so in a comment that nothing enforces. Once a vent can end at
    /// another TANK, both endpoints are holdups and the per-node loop must be
    /// able to ask which one owns the edge: the emitter skips it (its boil-off
    /// debits the inventory directly) and the receiver counts it as an inflow.
    /// Deriving that from the direction would be right today and silently wrong
    /// the moment a vent were ever stored the other way round, which is the
    /// shape of M12.1's own hardest bug.
    BoilOffVent { emitter: NodeId },
    /// A tank's overflow: tank → `Atmosphere`, built by the loader for EVERY
    /// tank (M23, docs/DESIGN.md §27 fork 2).
    ///
    /// Engine-written like a vent — the solve compiles it closed and reports
    /// zero, and `Engine::tick` writes whatever liquid stood above the brim at
    /// the end of the tick — but it is NOT a vent: it never carries `latent`,
    /// the boil-off model never writes it, and its far end is never a holdup.
    /// So it answers `is_engine_written` and not `is_boiloff_vent`, and an
    /// overflow is never counted as vapour arriving at a recovery drum.
    ///
    /// **`owner` is stored, for `BoilOffVent::emitter`'s reason.** The per-node
    /// loop also visits the atmosphere node the overflow ends at, and ownership
    /// derived from anything else is how M12.1 lost 2 539 kg.
    Overflow { owner: NodeId },
}

impl LeakRole {
    /// True for an edge whose flow the ENGINE writes rather than the hydraulic
    /// solve: a boil-off vent or a tank's overflow (M23, docs/DESIGN.md §27
    /// fork 3). One predicate, so the several passes that must skip such an
    /// edge — the solve's compile and its flows, the transport pass and the
    /// composition pass — cannot come to disagree about which edges those are.
    pub fn is_engine_written(&self) -> bool {
        matches!(
            self,
            LeakRole::BoilOffVent { .. } | LeakRole::Overflow { .. }
        )
    }

    /// True for a boil-off vent and nothing else. Since M23 this is NOT "the
    /// engine writes this edge" (that is `is_engine_written`, which an overflow
    /// also answers); it is kept for any site that means the vapour path.
    pub fn is_boiloff_vent(&self) -> bool {
        matches!(self, LeakRole::BoilOffVent { .. })
    }

    /// The tank that owns this overflow, or `None` on any other edge.
    ///
    /// The single owner of "whose overflow is this", read at the two sites that
    /// need it: the inflow loop skips the overflow its own tank owns, and the
    /// write site finds it (docs/DESIGN.md §27 fork 3).
    pub fn overflow_owner(&self) -> Option<NodeId> {
        match self {
            LeakRole::Overflow { owner } => Some(*owner),
            _ => None,
        }
    }

    /// The holdup that owns this vent, or `None` on any other edge.
    ///
    /// The single owner of "whose vent is this". Both readers go through it in
    /// opposite directions — `Engine::tick`'s inflow loop skips the vent whose
    /// emitter it IS and counts the one whose emitter it is not — so the two
    /// cannot come to disagree about ownership the way a `matches!` at each site
    /// could.
    pub fn boiloff_vent_emitter(&self) -> Option<NodeId> {
        match self {
            LeakRole::BoilOffVent { emitter } => Some(*emitter),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pipe {
    pub name: String,
    pub length: Meter,
    pub diameter: Meter,
    /// Darcy friction factor (constant for M1; Colebrook/Haaland later).
    pub friction_factor: f64,
    /// Elevation change target-minus-source [m], for the static head term.
    pub elevation_change: Meter,
    /// This edge's role in a declared leak path; `None` for an ordinary pipe.
    /// See `LeakRole` and docs/DESIGN.md §3b.
    #[serde(default)]
    pub leak: LeakRole,
    /// Ambient heat transfer coefficient × exposed area, `UA` [W/K].
    ///
    /// Spelled like `TankState::ambient_ua` and defaulting to ZERO for the same
    /// reason, but it does NOT drive the same equation. A tank is a lumped
    /// inventory, so its exchange is one signed `Q` added to its energy balance;
    /// a pipe is a flow-through body with no inventory at this fidelity, so its
    /// exchange is a TRANSFORM along the edge — see
    /// `energy::pipe_outlet_temperature`. Adding `UA·(T_AMBIENT − T)` to a pipe
    /// as if it were a tank would be dimensionally fine and physically wrong.
    ///
    /// The consequence worth flagging at the field: a pipe with a nonzero `UA`
    /// is NO LONGER ISOTHERMAL, which retires the identity M2.1 transport was
    /// built on (an edge's temperature is its upwind node's). Everything that
    /// reads a temperature off an edge must therefore say which END it means,
    /// and go through `energy::edge_temperature_at` to get it.
    #[serde(default = "no_ambient_exchange")]
    pub ambient_ua: WattPerKelvin,
    /// Transported material state, updated by the engine each tick.
    ///
    /// `stream.temperature` is the pipe's OUTLET temperature — the value the
    /// downstream node receives. With `ambient_ua = 0` and no dissipation the two
    /// ends agree and the distinction is invisible; otherwise it is a deliberate
    /// display choice, taken because the outlet is the one end a snapshot reader
    /// cannot reconstruct from the upwind node's temperature.
    ///
    /// That choice was originally justified by "nothing in the engine consumes
    /// this field", which M5.2 falsified: `network::compile_edge` reads it for a
    /// gas edge's transport density, and the outlet is the WRONG end for that —
    /// friction and ambient have already acted on it. The reader was corrected to
    /// take the upwind NODE's temperature where the node has one, so this field is
    /// once again display for every edge whose upwind endpoint is inertial. It is
    /// still consumed as the fallback when the upwind node is zero-volume and has
    /// no temperature of its own; see `compile_edge` for the size of what that
    /// costs and for what un-defers it.
    pub stream: Stream,
}

// ---------------------------------------------------------------------------
// Graph wrapper
// ---------------------------------------------------------------------------

/// `Clone` was derived here until M8.2 and had no call site in the workspace.
/// It went when `controls` arrived, because a `Box<dyn Controller>` owns state
/// and cloning a graph would silently fork a loop's memory into two engines that
/// then diverge — the opposite of what rule 3 wants from a copied plant. `Debug`
/// survives, which is why `Controller` carries it as a supertrait.
#[derive(Debug, Default)]
pub struct PlantGraph {
    g: StableDiGraph<Node, Pipe>,
    /// Thermal pairings between `HeatExchanger` sides. A `Vec`, not a map:
    /// insertion-ordered iteration is deterministic (rule 3), and the list is
    /// short enough that the linear `partner` lookup costs nothing.
    couplings: Vec<HeatExchangerCoupling>,
    /// The plant's regulating loops, beside the graph rather than on it
    /// (docs/DESIGN.md §10 fork 1). **Declaration order is execution order**, so
    /// this is a `Vec` and never a map (rule 3), and `LoopId` indexes it the way
    /// `NodeId` indexes the nodes.
    ///
    /// Empty for every scenario written before M8, which is what keeps them
    /// byte-identical: the loop pass iterates nothing and `Snapshot::controls`
    /// serializes nothing.
    controls: Vec<ControlLoop>,
    /// The plant's trips, beside the loops (docs/DESIGN.md §26 fork 1).
    /// Declaration order is `TripId` order and evaluation order.
    ///
    /// Empty for every scenario written before M22, which is what keeps them
    /// byte-identical: the trip pass takes an early return and
    /// `Snapshot::trips` serializes nothing.
    trips: Vec<Trip>,
}

impl PlantGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_node(&mut self, node: Node) -> NodeId {
        NodeId(self.g.add_node(node).index() as u32)
    }

    pub fn add_pipe(&mut self, from: NodeId, to: NodeId, pipe: Pipe) -> EdgeId {
        EdgeId(self.g.add_edge(from.into(), to.into(), pipe).index() as u32)
    }

    /// Does this plant have a node with this id? The check `Engine::apply` makes
    /// before a command touches the graph, because [`Self::node`] indexes
    /// directly and panics on an id it does not hold (M27, docs/DESIGN.md §8).
    pub fn has_node(&self, id: NodeId) -> bool {
        self.g.node_weight(NodeIndex::from(id)).is_some()
    }
    /// Does this plant have an edge with this id? See [`Self::has_node`].
    pub fn has_edge(&self, id: EdgeId) -> bool {
        self.g.edge_weight(EdgeIndex::from(id)).is_some()
    }

    pub fn node(&self, id: NodeId) -> &Node {
        &self.g[NodeIndex::from(id)]
    }
    pub fn node_mut(&mut self, id: NodeId) -> &mut Node {
        &mut self.g[NodeIndex::from(id)]
    }
    pub fn pipe(&self, id: EdgeId) -> &Pipe {
        &self.g[EdgeIndex::from(id)]
    }
    pub fn pipe_mut(&mut self, id: EdgeId) -> &mut Pipe {
        &mut self.g[EdgeIndex::from(id)]
    }

    /// (source, target) node ids of an edge.
    pub fn endpoints(&self, id: EdgeId) -> (NodeId, NodeId) {
        let (a, b) = self.g.edge_endpoints(id.into()).expect("valid edge id");
        (NodeId(a.index() as u32), NodeId(b.index() as u32))
    }

    /// Deterministic iteration: ascending index order, guaranteed stable.
    pub fn node_ids(&self) -> impl Iterator<Item = NodeId> + '_ {
        self.g.node_indices().map(|i| NodeId(i.index() as u32))
    }
    pub fn edge_ids(&self) -> impl Iterator<Item = EdgeId> + '_ {
        self.g.edge_indices().map(|i| EdgeId(i.index() as u32))
    }

    /// Edges incident to a node as (edge, other_node, is_incoming).
    pub fn incident(&self, id: NodeId) -> Vec<(EdgeId, NodeId, bool)> {
        use petgraph::Direction;
        let n: NodeIndex = id.into();
        let mut out = Vec::new();
        for dir in [Direction::Incoming, Direction::Outgoing] {
            for e in self.g.edges_directed(n, dir) {
                use petgraph::visit::EdgeRef;
                let other = if dir == Direction::Incoming {
                    e.source()
                } else {
                    e.target()
                };
                out.push((
                    EdgeId(e.id().index() as u32),
                    NodeId(other.index() as u32),
                    dir == Direction::Incoming,
                ));
            }
        }
        // Deterministic order regardless of petgraph internals.
        out.sort_by_key(|(e, _, _)| *e);
        out
    }

    /// The order in which `Engine::tick` must evaluate holdups, so that a vent's
    /// EMITTER is updated before its RECEIVER (M14, docs/DESIGN.md §16 fork 3).
    ///
    /// A boil-off vent's flow, composition, temperature and latent heat are
    /// written at the END of the emitting tank's own iteration, from the state
    /// the vapour left in. A tank receiving that vent must therefore be
    /// iterated after it, or it reads the PREVIOUS tick's vapour: a systematic
    /// one-tick lag, which parks `ṁ_v·dt` of mass in flight on every tick and
    /// puts a floor under any mass balance taken over the plant. Every ordinary
    /// edge in this engine debits and credits its two endpoints from the same
    /// flow inside one tick, and a vent is the only edge that could not.
    ///
    /// **Only tank → tank vents constrain anything.** A vent ending at an
    /// `Atmosphere` has no receiving holdup, so a plant on which every vent ends
    /// there — which is every plant written before M14 — produces an empty
    /// constraint set and takes the early return below. The order is then
    /// `node_ids()` exactly, by construction rather than by measurement, which
    /// is what keeps the existing corpus byte-identical.
    ///
    /// **A cycle is refused HERE, and that is the fork-5 refusal's real
    /// mechanism.** §16 fork 5 gives the reason as "a cycle makes the answer
    /// depend on node order"; with an evaluation order the sharper statement is
    /// that no such order exists, and Kahn's algorithm ending with nodes left
    /// over is what says so. Reachable from a scenario (`vent_to` naming a tank
    /// that vents back) and from a hand-built graph, and refused identically.
    ///
    /// Ties are broken by ascending `NodeId`, so the result is a function of the
    /// graph alone.
    pub fn holdup_evaluation_order(&self) -> Result<Vec<NodeId>, SimError> {
        use std::collections::{BTreeMap, BTreeSet};

        // (emitter, receiver) for every vent whose far end is itself a holdup.
        // An `Atmosphere` receiver constrains nothing: it has no branch in the
        // holdup update at all.
        let mut successors: BTreeMap<NodeId, BTreeSet<NodeId>> = BTreeMap::new();
        let mut constraints = 0usize;
        for eid in self.edge_ids() {
            let Some(emitter) = self.pipe(eid).leak.boiloff_vent_emitter() else {
                continue;
            };
            let (from, to) = self.endpoints(eid);
            let receiver = if from == emitter { to } else { from };
            if !matches!(self.node(receiver).kind, NodeKind::Tank(_)) {
                continue;
            }
            if successors.entry(emitter).or_default().insert(receiver) {
                constraints += 1;
            }
        }
        let all: Vec<NodeId> = self.node_ids().collect();
        if constraints == 0 {
            return Ok(all);
        }

        // Kahn's algorithm with a lowest-id tie-break. With no constraints this
        // reproduces `node_ids()` exactly, which is why the early return above
        // is a shortcut rather than a special case.
        let mut indegree: BTreeMap<NodeId, usize> = all.iter().map(|n| (*n, 0)).collect();
        for targets in successors.values() {
            for t in targets {
                *indegree
                    .get_mut(t)
                    .expect("receiver is a node of this graph") += 1;
            }
        }
        let mut ready: BTreeSet<NodeId> = indegree
            .iter()
            .filter(|(_, d)| **d == 0)
            .map(|(n, _)| *n)
            .collect();
        let mut order = Vec::with_capacity(all.len());
        while let Some(next) = ready.iter().next().copied() {
            ready.remove(&next);
            order.push(next);
            if let Some(targets) = successors.get(&next) {
                for t in targets {
                    let d = indegree
                        .get_mut(t)
                        .expect("receiver is a node of this graph");
                    *d -= 1;
                    if *d == 0 {
                        ready.insert(*t);
                    }
                }
            }
        }
        if order.len() != all.len() {
            let stuck: Vec<&str> = all
                .iter()
                .filter(|n| !order.contains(n))
                .map(|n| self.node(*n).name.as_str())
                .collect();
            return Err(SimError::Scenario(format!(
                "the boil-off vents on these holdups form a cycle: {}. Each vent is written                  at the end of its own tank's update and read by the tank it arrives at, so                  the tanks have to be evaluated emitter-first — and a cycle has no such                  order (docs/DESIGN.md §16 fork 5)",
                stuck.join(", ")
            )));
        }
        Ok(order)
    }

    pub fn node_count(&self) -> usize {
        self.g.node_count()
    }
    pub fn edge_count(&self) -> usize {
        self.g.edge_count()
    }

    /// Thermally pair two `HeatExchanger` sides. Validation of the node kinds
    /// and of ε belongs to the loader, which can name the offending scenario
    /// entry; this is the plain storage operation.
    pub fn add_coupling(&mut self, coupling: HeatExchangerCoupling) {
        self.couplings.push(coupling);
    }

    pub fn couplings(&self) -> &[HeatExchangerCoupling] {
        &self.couplings
    }

    /// The other side of `id`'s exchanger, with the pair's effectiveness.
    ///
    /// Searches both fields, so a coupling may be declared in either order and
    /// no caller has to know whether a given node was written as side A or B.
    pub fn exchanger_partner(&self, id: NodeId) -> Option<(NodeId, f64)> {
        self.couplings.iter().find_map(|c| {
            if c.side_a == id {
                Some((c.side_b, c.effectiveness))
            } else if c.side_b == id {
                Some((c.side_a, c.effectiveness))
            } else {
                None
            }
        })
    }

    pub fn find_node(&self, name: &str) -> Option<NodeId> {
        self.node_ids().find(|id| self.node(*id).name == name)
    }

    /// Append a control loop. Its `LoopId` is its position, so declaration order
    /// in the scenario file is the id order — and the execution order within each
    /// half of the tick's control pass, where cascade primaries run before every
    /// other loop (M25, docs/DESIGN.md §29 fork 3).
    ///
    /// Validation of the two node kinds it names, and of one-writer-per-actuator,
    /// belongs to the loader, which can name the offending `[[controls]]` entry;
    /// this is the plain storage operation, exactly like `add_coupling`.
    pub fn add_control(&mut self, control: ControlLoop) -> LoopId {
        self.controls.push(control);
        LoopId(self.controls.len() as u32 - 1)
    }

    pub fn controls(&self) -> &[ControlLoop] {
        &self.controls
    }

    /// The loops, mutably — the engine's loop pass writes `last_measurement`,
    /// `last_output` and (from M8.3) the algorithm's own state through this.
    pub fn controls_mut(&mut self) -> &mut [ControlLoop] {
        &mut self.controls
    }

    /// One loop by id, or `None` if the id names no loop.
    ///
    /// `Option` rather than an index panic: a `LoopId` arrives from outside the
    /// engine on `Command::SetSetpoint`, so an out-of-range one is a command to
    /// refuse, not a bug to crash on (rule 5).
    pub fn control(&self, id: LoopId) -> Option<&ControlLoop> {
        self.controls.get(id.0 as usize)
    }

    pub fn control_mut(&mut self, id: LoopId) -> Option<&mut ControlLoop> {
        self.controls.get_mut(id.0 as usize)
    }

    /// Append a trip. Its `TripId` is its position. Validation belongs to the
    /// loader, which can name the offending `[[trips]]` entry.
    pub fn add_trip(&mut self, trip: Trip) -> TripId {
        self.trips.push(trip);
        TripId(self.trips.len() as u32 - 1)
    }

    pub fn trips(&self) -> &[Trip] {
        &self.trips
    }

    pub fn trips_mut(&mut self) -> &mut [Trip] {
        &mut self.trips
    }

    /// One trip by id, or `None` if the id names no trip — `Option` for
    /// `control`'s reason: the id arrives from outside the engine on
    /// `Command::ResetTrip`.
    pub fn trip(&self, id: TripId) -> Option<&Trip> {
        self.trips.get(id.0 as usize)
    }

    pub fn trip_mut(&mut self, id: TripId) -> Option<&mut Trip> {
        self.trips.get_mut(id.0 as usize)
    }

    /// The first LATCHED trip that holds `node`, if any.
    ///
    /// The one question every refusal in `Engine::apply` asks (docs/DESIGN.md
    /// §26 fork 4). "First" is enough: the loader refuses two trips that give
    /// one valve different positions, so every latched trip on a node demands
    /// the same safe state of it, and a pump's is always "stopped".
    pub fn latched_trip_on(&self, node: NodeId) -> Option<&Trip> {
        self.trips
            .iter()
            .find(|t| t.state.is_tripped() && t.acts_on(node))
    }

    /// Is `value` a legal setpoint for a loop measuring at `point`?
    ///
    /// **One owner, called from both write points**: the loader, when a
    /// `[[controls]]` entry declares a setpoint, and `Engine::apply`, when
    /// `Command::SetSetpoint` moves one. Two copies of "what is a reachable
    /// target" could disagree, and the disagreement would look like a loop that
    /// loaded fine and then refused the number it was loaded with.
    ///
    /// The bound is the measured node's own geometry rather than a constant: a
    /// level setpoint above the tank's height is a target the plant cannot reach,
    /// so the loop sits pinned at saturation and reads as a tuning problem. `0`
    /// stays legal — it is "drain it".
    ///
    /// # Errors
    /// `SimError::InvalidCommand` naming the range, or if the node cannot answer
    /// for that variable at all.
    pub fn check_setpoint(
        &self,
        point: MeasurementPoint,
        value: ControlledValue,
    ) -> Result<(), SimError> {
        let node = match (point, value) {
            // **Finite and strictly positive, with no upper bound** (docs/DESIGN.md
            // §24 fork 4). Zero is "shut the valve", which is a MANUAL action and
            // not a regulation; a negative setpoint names flow against the pipe's
            // declared direction, which a series valve cannot regulate (E11). The
            // reachable flow is SOLVED, so a setpoint above it is not refused: it
            // pins the valve open, and the anti-windup clamp is what handles that.
            (MeasurementPoint::Pipe(pipe), ControlledValue::Flow { kg_per_s }) => {
                if !kg_per_s.is_finite() || kg_per_s.value() <= 0.0 {
                    return Err(SimError::InvalidCommand(format!(
                        "flow setpoint {} kg/s on pipe '{}' is not a finite flow above zero. \
                         Zero is \"shut the valve\", a MANUAL action rather than a regulation, \
                         and a negative flow runs against the pipe's declared direction, which \
                         a valve in series with it cannot regulate (docs/DESIGN.md §24 fork 4)",
                        kg_per_s.value(),
                        self.pipe(pipe).name
                    )));
                }
                return Ok(());
            }
            (MeasurementPoint::Pipe(pipe), _) => {
                return Err(SimError::InvalidCommand(format!(
                    "pipe '{}' carries a flow and nothing else, so it has no {:?} setpoint \
                     range",
                    self.pipe(pipe).name,
                    value.variable()
                )))
            }
            (MeasurementPoint::Node(node), _) => node,
        };
        match (value, &self.node(node).kind) {
            (ControlledValue::Level { m }, NodeKind::Tank(t)) => {
                if !m.is_finite() || m.value() < 0.0 || m.value() > t.height.value() {
                    return Err(SimError::InvalidCommand(format!(
                        "level setpoint {} m is outside tank '{}''s range [0, {}] m — a setpoint the \
                         plant cannot reach leaves the loop pinned at saturation",
                        m.value(),
                        self.node(node).name,
                        t.height.value()
                    )));
                }
                Ok(())
            }
            (ControlledValue::Level { .. }, _) => Err(SimError::InvalidCommand(format!(
                "node '{}' is not a tank, so it has no level setpoint range",
                self.node(node).name
            ))),
            // **A vessel has no geometric analogue of a tank's height, and the
            // reflex that wants one is rejected in the note** (docs/DESIGN.md §12
            // fork 2). `P = m·R·T/(V·M̄)` is unbounded above; there is no height
            // to exceed. Reconstructing reachability from the plant — refuse a
            // setpoint above every source pressure — is possible from `&self` and
            // is a network traversal masquerading as a range check: wrong the
            // moment a plant has a compressor or a second source, and refusing
            // legitimate plants for a reason the author cannot act on.
            //
            // So the bound is finiteness and strict positivity, and the asymmetry
            // with the level arm is the part worth stating: `0` is LEGAL there and
            // refused here. A level setpoint of zero is "drain it"; a pressure
            // setpoint of zero is a vacuum the ideal-gas relation cannot reach at
            // any finite mass, so a loop given one sits pinned at saturation
            // forever, which is exactly what a setpoint bound exists to prevent.
            (ControlledValue::Pressure { pa }, NodeKind::Vessel(_)) => {
                if !pa.is_finite() || pa.value() <= 0.0 {
                    return Err(SimError::InvalidCommand(format!(
                        "pressure setpoint {} Pa on vessel '{}' is not a finite positive absolute \
                         pressure. A vessel has no geometric bound the way a tank's height bounds a \
                         level, so this is the whole of the check — but 0 is refused where a level's \
                         0 is legal: `P = m·R·T/(V·M̄)` reaches zero only at zero mass, so the loop \
                         would sit pinned at saturation",
                        pa.value(),
                        self.node(node).name
                    )));
                }
                Ok(())
            }
            (ControlledValue::Pressure { .. }, _) => Err(SimError::InvalidCommand(format!(
                "node '{}' is not a vessel, so it has no pressure setpoint range",
                self.node(node).name
            ))),
            // **Finiteness and `> 0 K` only, and the bound a reader expects is
            // refused on purpose** (docs/DESIGN.md §21 fork 6). A boiling tank is
            // parked on its bubble point, so a setpoint above it is unreachable —
            // but the bubble point moves with composition, which makes it a state
            // and not a range. A temperature has no geometric bound the way a
            // level has a height; absolute zero is the only one it has.
            // A furnace or cooler OUTLET takes the same bound (M19): an outlet
            // has no geometry either, and its ceiling is a function of the flow.
            (
                ControlledValue::Temperature { k },
                NodeKind::Tank(_)
                | NodeKind::Vessel(_)
                | NodeKind::Furnace { .. }
                | NodeKind::Cooler { .. },
            ) => {
                if !k.is_finite() || k.value() <= 0.0 {
                    return Err(SimError::InvalidCommand(format!(
                        "temperature setpoint {} K on '{}' is not a finite temperature above \
                         absolute zero",
                        k.value(),
                        self.node(node).name
                    )));
                }
                Ok(())
            }
            (ControlledValue::Temperature { .. }, _) => Err(SimError::InvalidCommand(format!(
                "node '{}' is neither a holdup nor a furnace or cooler outlet, so it has no \
                 temperature setpoint range",
                self.node(node).name
            ))),
            (ControlledValue::Flow { .. }, _) => Err(SimError::InvalidCommand(format!(
                "node '{}' is not a pipe, so it has no flow setpoint range: a flow belongs to a \
                 pipe",
                self.node(node).name
            ))),
        }
    }

    /// Read one regulated variable off the plant, as it stands right now.
    ///
    /// **The single owner of "where does a control loop's measurement come
    /// from", and the answer differs by variable** (docs/DESIGN.md §10 fork 3,
    /// which originally got this wrong for the one variable M8.2 builds). A level
    /// is a STORED quantity: `TankState::mass` lives on this graph and is real
    /// from load, so a level loop reads the graph and has a genuine measurement
    /// at tick 0. It does NOT read `energy::NodeStates`, which carries
    /// temperature, composition, reactor duties and column separations and no
    /// inventory at all, and which is empty before the first tick.
    ///
    /// **A `Vessel`'s pressure is stored too, and the sentence that used to stand
    /// here said otherwise** (docs/DESIGN.md §12). It read: "a pressure or a
    /// temperature is SOLVED and lives in `last_solution` / `NodeStates`; a loop
    /// on either has no measurement at tick 0 and needs a stated rule for that
    /// tick." `VesselState::pressure` is `m/C` and `m` is on this graph — real
    /// from load, and *exactly* the declared figure, because `build_engine`
    /// computes the initial mass as `P_declared · capacitance` through the same
    /// `capacitance` method `pressure` divides by, precisely so that round trip is
    /// exact. So this signature did not change and the promised tick-0 rule was
    /// never owed.
    ///
    /// The claim is true of a `Junction`, whose pressure is an unknown of the
    /// solve. The tick-0 rule it lacked now exists (below, M19) and is not applied
    /// to a pressure: the junction arm refuses as a scope decision, naming
    /// `docs/DEFERRED.md` E9.
    ///
    /// **A holdup's temperature is stored as well, and this doc used to say it
    /// "really is a `NodeStates` quantity"** (docs/DESIGN.md §21 — the fourth
    /// recurrence of the one error). `TankState::temperature` and
    /// `VesselState::temperature` are on this graph, set from `temperature_c` at
    /// load and written back every tick. A snapshot's `temperature_k` for the
    /// same tank is one Euler step behind this reading, exactly, which is the
    /// shape M8.5 found for a tank's pressure against its mass.
    ///
    /// **What IS a `NodeStates` quantity is a zero-volume node's temperature, and
    /// for a `Furnace` or `Cooler` outlet this reader answers from `resolved`**
    /// (M19, docs/DESIGN.md §23). That premise was checked before it was built on,
    /// because it had been false four times, and it is true: an outlet has no
    /// field on this graph, is resolved by the sweep into the engine's
    /// `NodeStates`, and does not exist before the first tick. So this is the one
    /// reading that can be ABSENT, and `Ok(None)` is how it says so — never a
    /// stand-in (the feed's temperature, ambient, the setpoint), because a PI loop
    /// seeds its memory against whatever it first measures. The rule the caller
    /// applies is "no measurement, no action" (§23 fork 2). An outlet is also
    /// absent while it is STAGNANT: a zero-volume node with no inflow holds a
    /// placeholder, which `NodeStates::held` records and this reader refuses to
    /// pass off as a measurement (§23 fork 4).
    ///
    /// `resolved` is the last tick's states — exactly what the engine will hand
    /// the next solve — and empty at load, which is the truth at load. Every other
    /// arm ignores it, so the level, pressure and holdup-temperature readings
    /// cannot have moved.
    ///
    /// **A pipe's flow is read from `hydraulics`, the last hydraulic solution, and
    /// NEVER from `Pipe::stream`** (M20, docs/DESIGN.md §24 fork 2). The stream's
    /// `mass_flow` is on the graph, but at load it holds the initialiser zero
    /// `Stream::stagnant` wrote — not a declaration, and indistinguishable from a
    /// valve shut on tick 400 — so a flow is ABSENT at load exactly as an outlet
    /// is, and `hydraulics` is `None` there, which is the truth. Once a solve has
    /// run, the solution's number is the one step 2 of the tick copied onto the
    /// stream, so the two agree bit for bit from then on (§24 gate 2). A flow of
    /// zero is a MEASUREMENT: the solve computes it, so there is no `held` rule
    /// here, and none may be borrowed from the outlet arm. A negative flow is
    /// returned as measured, never clipped (E11). Every node arm ignores this
    /// argument.
    ///
    /// Called at load to seed `ControlLoop::last_measurement` and once per loop
    /// per tick thereafter, so a loop's seeded measurement and its running one
    /// can never be taken by two different rules.
    ///
    /// # Errors
    /// `SimError::Scenario` if the node cannot answer for that variable — a level
    /// asked of anything that is not a `Tank`, a pressure asked of anything that
    /// is not a `Vessel`. The loader calls this very function at load, so these
    /// messages ARE the user-facing ones rather than a rule-5 backstop behind
    /// them: `build_controls` asks the graph instead of matching the kind itself,
    /// so a kind the loader accepted and this reader then rejected is not a state
    /// that can exist.
    pub fn measure(
        &self,
        slate: &Slate,
        resolved: &crate::energy::NodeStates,
        hydraulics: Option<&crate::traits::HydraulicSolution>,
        point: MeasurementPoint,
        variable: MeasuredVariable,
    ) -> Result<Option<ControlledValue>, SimError> {
        let node = match (point, variable) {
            (MeasurementPoint::Pipe(pipe), MeasuredVariable::Flow) => {
                let Some(solution) = hydraulics else {
                    return Ok(None);
                };
                // A solution that exists and omits a pipe is an engine fault,
                // not an absence: `None` would hold the loop silently, and step
                // 2 of the tick already refuses the same omission.
                let flow = solution.edge_mass_flow.get(&pipe).ok_or_else(|| {
                    SimError::Numerical(format!(
                        "the hydraulic solution has no flow for measured pipe '{}'",
                        self.pipe(pipe).name
                    ))
                })?;
                return Ok(Some(ControlledValue::Flow {
                    kg_per_s: KgPerSec(*flow),
                }));
            }
            (MeasurementPoint::Pipe(pipe), _) => {
                return Err(SimError::Scenario(format!(
                    "pipe '{}' carries a flow and holds nothing, so it has no {} to control. \
                     A level, a pressure and a temperature are measured at a NODE: write \
                     `node = \"…\"` in `measurement`, not `pipe` (docs/DESIGN.md §24 fork 1)",
                    self.pipe(pipe).name,
                    variable.noun()
                )))
            }
            (MeasurementPoint::Node(node), _) => node,
        };
        match (variable, &self.node(node).kind) {
            (MeasuredVariable::Level, NodeKind::Tank(t)) => Ok(Some(ControlledValue::Level {
                m: t.level(slate),
            })),
            (MeasuredVariable::Level, _) => Err(SimError::Scenario(format!(
                "node '{}' is not a tank, so it has no level to control. A level names nothing on a \
                 vessel whose state IS pressure (docs/DESIGN.md §3a fork 2)",
                self.node(node).name
            ))),
            // Stored, exact from load, per the note above.
            (MeasuredVariable::Pressure, NodeKind::Vessel(v)) => {
                Ok(Some(ControlledValue::Pressure {
                    pa: v.pressure(slate),
                }))
            }
            // **Refused, and the reason is measured rather than stylistic**
            // (docs/DESIGN.md §12 fork 1). A tank's pressure is
            // `P_atm + ρ·g·h = P_atm + m·g/A`: the density cancels exactly (M8.5),
            // so a tank-pressure loop is a level loop in a worse unit — a setpoint
            // the author has to convert by hand and a gain in the wrong reciprocal
            // unit. The message names what was meant instead.
            (MeasuredVariable::Pressure, NodeKind::Tank(_)) => Err(SimError::Scenario(format!(
                "node '{}' is a tank, and a tank's pressure is `P_atm + ρ·g·h = P_atm + m·g/A` — the \
                 density cancels exactly, so controlling it is controlling the LEVEL in a worse \
                 unit. Write `variable = \"level\"` with `setpoint_m`",
                self.node(node).name
            ))),
            // **This is the one node kind fork 3's claim is actually true of.** A
            // junction holds nothing; its pressure is an unknown of the solve and
            // does not exist before the first tick. It used to be refused for the
            // want of a tick-0 rule, and then (M19) for the want of the solution
            // being passed in here. M20 passes it — `hydraulics`, for a pipe's
            // flow — so NEITHER reason stands: the rule exists, and so does the
            // path. The refusal is a scope decision and nothing more (E9): nothing
            // asks for junction-pressure control.
            (MeasuredVariable::Pressure, NodeKind::Junction) => Err(SimError::Scenario(format!(
                "node '{}' is a junction, which holds nothing: its pressure is an UNKNOWN of the \
                 network solve, absent before the first tick. Both halves of reading one exist \
                 — the rule for a measurement that does not exist yet (docs/DESIGN.md §23: no \
                 measurement, no action) and the last hydraulic solution, which a pipe's flow is \
                 read from (§24). So neither is what is missing: junction-pressure control is \
                 not built, as a scope decision (docs/DEFERRED.md E9)",
                self.node(node).name
            ))),
            // The catch-all, and its reason is a third distinct one: a source's
            // and a sink's pressures are pinned by declaration, which makes them
            // boundary conditions rather than states. Regulating one is regulating
            // the scenario file.
            (MeasuredVariable::Pressure, _) => Err(SimError::Scenario(format!(
                "node '{}' has no pressure of its own to control. A vessel's pressure is a STATE; a \
                 source's and a sink's are pinned by declaration, so a loop on one would be \
                 regulating the scenario file rather than the plant",
                self.node(node).name
            ))),
            // Stored on the graph and exact from load (docs/DESIGN.md §21 fork 1).
            // The vessel arm is argued rather than inherited: same storage, same
            // write-back, and refusing it would be a refusal whose only reason is
            // that no demo exercises it.
            (MeasuredVariable::Temperature, NodeKind::Tank(t)) => {
                Ok(Some(ControlledValue::Temperature { k: t.temperature }))
            }
            (MeasuredVariable::Temperature, NodeKind::Vessel(v)) => {
                Ok(Some(ControlledValue::Temperature { k: v.temperature }))
            }
            // **The furnace or cooler OUTLET — the one reading that can be absent**
            // (docs/DESIGN.md §23). Resolved by the last tick's sweep, so absent
            // before the first one, and absent while the unit is stagnant, when
            // its entry is a held placeholder rather than a computed value. Both
            // absences are `None`, and the caller holds the loop on either.
            (MeasuredVariable::Temperature, NodeKind::Furnace { .. } | NodeKind::Cooler { .. }) => {
                if resolved.held.contains(&node) {
                    return Ok(None);
                }
                Ok(resolved
                    .temperature
                    .get(&node)
                    .map(|&k| ControlledValue::Temperature { k }))
            }
            // **Zero-volume like the two above, and refused as a SCOPE decision,
            // not for the want of a tick-0 rule** (§23 fork 5). The rule applies
            // to them unchanged; no plant or fixture exercises one, and an
            // admission nobody runs is untested. An exchanger side is further
            // off: it resolves in the coupled-pair branch of the sweep, where the
            // stagnant-outlet set would have to be argued separately.
            (
                MeasuredVariable::Temperature,
                NodeKind::Junction
                | NodeKind::Pump { .. }
                | NodeKind::Valve { .. }
                | NodeKind::HeatExchanger,
            ) => Err(SimError::Scenario(format!(
                "node '{}' holds no volume, so its temperature is resolved by the tick, and a \
                 zero-volume temperature is measured only at a FURNACE or COOLER outlet \
                 (docs/DESIGN.md §23). Other zero-volume nodes are not admitted, as a scope \
                 decision (docs/DEFERRED.md E9). Measure the furnace or cooler itself, or the \
                 holdup the stream runs into",
                self.node(node).name
            ))),
            // Its own reason: a relief valve is SHUT in normal operation, so its
            // temperature is the no-inflow placeholder nearly always, and under
            // §23 fork 4 a loop on it would hold forever and never act.
            (MeasuredVariable::Temperature, NodeKind::ReliefValve { .. }) => {
                Err(SimError::Scenario(format!(
                    "node '{}' is a relief valve, which is shut in normal operation: with no flow \
                     through it its temperature is a held placeholder rather than a resolved \
                     value, so a loop measuring it would hold forever without acting \
                     (docs/DESIGN.md §23 fork 5)",
                    self.node(node).name
                )))
            }
            // A column's tray temperatures are the separation model's output, and a
            // reactor's outlet is `t_set` — a declared temperature the unit already
            // holds exactly, so a loop on it would regulate a number that is
            // already regulated.
            (MeasuredVariable::Temperature, NodeKind::Column { .. } | NodeKind::Reactor { .. }) => {
                Err(SimError::Scenario(format!(
                    "node '{}' is a column or a reactor: it holds no temperature of its own for a \
                     loop to regulate. A column's temperatures are its separation model's \
                     output, and a reactor's outlet is its declared `t_set_c`, already held \
                     exactly (docs/DESIGN.md §21 fork 1)",
                    self.node(node).name
                )))
            }
            (
                MeasuredVariable::Temperature,
                NodeKind::Source { .. } | NodeKind::Sink { .. } | NodeKind::Atmosphere,
            ) => Err(SimError::Scenario(format!(
                "node '{}' is a boundary: its temperature is pinned by declaration, so a loop on \
                 it would be regulating the scenario file rather than the plant",
                self.node(node).name
            ))),
            // A node conducts; a flow belongs to one of its pipes. Naming the valve
            // node itself was the rejected fork-1(b) (docs/DESIGN.md §24).
            (MeasuredVariable::Flow, _) => Err(SimError::Scenario(format!(
                "node '{}' is a node, and a flow belongs to a PIPE: write `pipe = \"…\"` in \
                 `measurement`, not `node`, naming one of the actuating valve's own two pipes \
                 (docs/DESIGN.md §24 fork 1)",
                self.node(node).name
            ))),
        }
    }

    /// The name a faceplate or a refusal gives a measurement point: the node's
    /// name or the pipe's.
    pub fn point_name(&self, point: MeasurementPoint) -> &str {
        match point {
            MeasurementPoint::Node(node) => &self.node(node).name,
            MeasurementPoint::Pipe(pipe) => &self.pipe(pipe).name,
        }
    }

    /// Where a loop's actuator stands, as a fraction of the loop's authority.
    ///
    /// **The single owner of the actuator side, as `measure` is of the
    /// measurement side** (docs/DESIGN.md §21). Three sites read this quantity —
    /// the loader's seed of `last_output`, pass 1 of the tick's control pass, and
    /// the MANUAL→AUTO transfer — and if they took it by two rules, a transfer
    /// would seed from one notion of position while the tick ran on another.
    ///
    /// A valve's position is its opening, read bare: no arithmetic on the valve
    /// path, so the level and pressure loops that predate M17 read exactly what
    /// they read before. A cooler's or a furnace's is `duty / max_duty` — the
    /// same map for both duty actuators, whichever way the loop acts, so the
    /// faceplate reads the real firing or cooling fraction (docs/DESIGN.md §22
    /// fork 1 rejects the inverted `(1 − u)·max` map on exactly that ground).
    ///
    /// **A cascade primary's position is its secondary's SETPOINT** (M25,
    /// docs/DESIGN.md §29 fork 2), as a fraction of the primary's declared range:
    /// `(sp − min)/(max − min)`. That is the fork's whole argument — MANUAL
    /// tracking, the MANUAL→AUTO seed, the clamp's back-calculation and the
    /// open-cascade re-seed all go through this reader, so none of them needed new
    /// code for a primary.
    ///
    /// # Errors
    /// `SimError::Scenario` if the pairing of actuator and range is not one the
    /// loader builds — a valve with a duty range, a cooler or furnace without
    /// one, a loop without a setpoint range (or a node with one), or any other
    /// kind. Reachable only from a hand-built graph, and said rather than
    /// answered with an invented position (rule 5).
    pub fn actuator_position(
        &self,
        actuator: Actuator,
        max_duty: Option<Watt>,
        range: Option<SetpointRange>,
    ) -> Result<f64, SimError> {
        let actuator = match (actuator, range) {
            (Actuator::Node(node), None) => node,
            (Actuator::Loop(id), Some(range)) => {
                let secondary = self.control(id).ok_or_else(|| unpaired_loop(id))?;
                return Ok(range.position(secondary.setpoint));
            }
            (actuator, _) => return Err(unranged_actuator(self, actuator)),
        };
        match (&self.node(actuator).kind, max_duty) {
            (NodeKind::Valve { opening, .. }, None) => Ok(*opening),
            (NodeKind::Cooler { duty } | NodeKind::Furnace { duty }, Some(max)) => {
                Ok(duty.value() / max.value())
            }
            _ => Err(unpaired_actuator(&self.node(actuator).name, max_duty)),
        }
    }

    /// Put a loop's actuator at `position`, a fraction of the loop's authority —
    /// the inverse of `actuator_position`, and its only writer.
    ///
    /// A valve's opening is `position` itself; a cooler's or a furnace's duty is
    /// `position · max_duty`; a cascade secondary's setpoint is
    /// `SetpointRange::setpoint_at(position)`, clamped to the range.
    ///
    /// # Errors
    /// As `actuator_position`.
    pub fn set_actuator_position(
        &mut self,
        actuator: Actuator,
        max_duty: Option<Watt>,
        range: Option<SetpointRange>,
        position: f64,
    ) -> Result<(), SimError> {
        let actuator = match (actuator, range) {
            (Actuator::Node(node), None) => node,
            (Actuator::Loop(id), Some(range)) => {
                let secondary = self.control_mut(id).ok_or_else(|| unpaired_loop(id))?;
                secondary.setpoint = range.setpoint_at(position);
                return Ok(());
            }
            (actuator, _) => return Err(unranged_actuator(self, actuator)),
        };
        match (&mut self.node_mut(actuator).kind, max_duty) {
            (NodeKind::Valve { opening, .. }, None) => {
                *opening = position;
                Ok(())
            }
            (NodeKind::Cooler { duty } | NodeKind::Furnace { duty }, Some(max)) => {
                *duty = max * position;
                Ok(())
            }
            _ => Err(unpaired_actuator(&self.node(actuator).name, max_duty)),
        }
    }

    /// The loop driving `secondary` as its cascade primary, if any (M25).
    ///
    /// One at most: the loader refuses two primaries on one secondary, by the
    /// rule that refuses two loops on one valve.
    pub fn primary_of(&self, secondary: LoopId) -> Option<&ControlLoop> {
        self.controls
            .iter()
            .find(|c| c.actuator == Actuator::Loop(secondary))
    }
}

/// The refusal both actuator accessors share for a cascade link to no loop.
fn unpaired_loop(id: LoopId) -> SimError {
    SimError::Scenario(format!(
        "a cascade primary drives {id:?}, which names no loop on this plant"
    ))
}

/// The refusal both actuator accessors share when a setpoint range is on the
/// wrong kind of actuator: present on a node, or absent on a loop.
fn unranged_actuator(graph: &PlantGraph, actuator: Actuator) -> SimError {
    SimError::Scenario(match actuator {
        Actuator::Node(node) => format!(
            "control loop actuator '{}' is a node, and a setpoint range is a cascade \
             primary's authority over another loop's setpoint (docs/DESIGN.md §29 fork 2)",
            graph.node(node).name
        ),
        Actuator::Loop(id) => format!(
            "a cascade primary drives {id:?} with no setpoint range: its output is a \
             fraction of that range, so without one it stands for no setpoint"
        ),
    })
}

/// The refusal both actuator accessors share: a pairing the loader never builds.
fn unpaired_actuator(name: &str, max_duty: Option<Watt>) -> SimError {
    SimError::Scenario(format!(
        "control loop actuator '{name}' is not an actuator this loop can position: a loop writes \
         a valve's opening (with no duty range) or a cooler's or furnace's duty (with one), and \
         this pairing is {}",
        if max_duty.is_some() {
            "a duty range on a node that is neither a cooler nor a furnace"
        } else {
            "no duty range, on a node that is not a valve"
        }
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The next representable value above a positive `x`, and below it.
    fn up(x: f64) -> f64 {
        f64::from_bits(x.to_bits() + 1)
    }
    fn down(x: f64) -> f64 {
        f64::from_bits(x.to_bits() - 1)
    }

    fn pressure(pa: f64) -> ControlledValue {
        ControlledValue::Pressure { pa: Pascal(pa) }
    }

    /// docs/DESIGN.md §26 gate 3's unit half: the comparison itself, at an exact
    /// tie and one ULP either side, with no loader round trip in the way. A high
    /// trip fires at `≥`, a low one at `≤`, and the tie fires both. The reset
    /// test is `!reached`, so a trip may be reset exactly on the far side of the
    /// tie and not at it.
    #[test]
    fn a_trip_fires_at_its_limit_and_one_ulp_inside_it_does_not() {
        let limit = pressure(1.2e6);
        for (measured, high, low) in [
            (1.2e6, true, true),
            (up(1.2e6), true, false),
            (down(1.2e6), false, true),
        ] {
            assert_eq!(
                TripDirection::High
                    .reached(pressure(measured), limit)
                    .unwrap(),
                high,
                "high trip at {measured:e} against 1.2e6"
            );
            assert_eq!(
                TripDirection::Low
                    .reached(pressure(measured), limit)
                    .unwrap(),
                low,
                "low trip at {measured:e} against 1.2e6"
            );
        }
    }

    /// A NaN compares false both ways, which would read as "safe" on either
    /// direction; and a mismatched variable is a dimensional error. Both are
    /// refusals, not verdicts.
    #[test]
    fn a_trip_refuses_to_compare_what_it_cannot() {
        let limit = pressure(1.2e6);
        assert!(TripDirection::High
            .reached(pressure(f64::NAN), limit)
            .is_err());
        assert!(TripDirection::Low
            .reached(pressure(f64::NAN), limit)
            .is_err());
        let level = ControlledValue::Level { m: Meter(1.2e6) };
        assert!(TripDirection::High.reached(level, limit).is_err());
    }

    /// docs/DESIGN.md §29 fork 2: a cascade primary's range map round-trips inside
    /// `[0, 1]` at both ends, on the demo's kelvin range and on a flow range. At
    /// `u = 1` the unclamped sum `min + u·(max − min)` need not round to `max`,
    /// and a setpoint one ULP past it reads back as `1.0000000000000002` — a
    /// position `seed_from_output` refuses on the tick a cascade opens. The sweep
    /// also takes every position a near-1 output can be, in both directions.
    #[test]
    fn a_setpoint_range_round_trips_inside_the_unit_interval_at_both_ends() {
        let kelvin = SetpointRange {
            min: ControlledValue::Temperature {
                k: Kelvin(40.0 + 273.15),
            },
            max: ControlledValue::Temperature {
                k: Kelvin(65.0 + 273.15),
            },
        };
        let flow = SetpointRange {
            min: ControlledValue::Flow {
                kg_per_s: KgPerSec(0.3),
            },
            max: ControlledValue::Flow {
                kg_per_s: KgPerSec(17.1),
            },
        };
        for range in [kelvin, flow] {
            assert_eq!(range.setpoint_at(0.0), range.min, "u = 0 is the bottom");
            assert_eq!(
                range.setpoint_at(1.0),
                range.max,
                "u = 1 is the top, exactly"
            );
            assert_eq!(range.position(range.min), 0.0);
            assert_eq!(range.position(range.max), 1.0);
            let mut u = 1.0_f64;
            let mut v = 0.0_f64;
            for _ in 0..64 {
                for x in [u, v] {
                    let back = range.position(range.setpoint_at(x));
                    assert!(
                        (0.0..=1.0).contains(&back),
                        "{range:?}: u = {x:e} reads back as {back:e}"
                    );
                    assert!(range.contains(range.setpoint_at(x)));
                }
                u = down(u);
                v = up(v.max(f64::MIN_POSITIVE));
            }
        }
        // The clamp is load bearing on an end, not decoration: a position past
        // 1 writes the top, and one below 0 the bottom.
        assert_eq!(kelvin.setpoint_at(up(1.0)), kelvin.max);
        assert_eq!(kelvin.setpoint_at(-1e-12), kelvin.min);
    }
}
