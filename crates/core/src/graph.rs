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
/// One variant, and that is the M8.2 scope (docs/DESIGN.md §10 fork 0): a tank's
/// level, because both halves of it already exist and are already load-bearing,
/// so the slice adds no measurement path and no actuator alongside the control
/// machinery it is there to test.
///
/// Pressure, temperature and flow are deferred per-variable, each needing a
/// measurement path and an actuator that exists. Pressure is the near one — see
/// the note's deferral list.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MeasuredVariable {
    Level,
}

impl MeasuredVariable {
    /// The scenario key that carries this variable's setpoint, unit included.
    ///
    /// Used only to phrase the loader's refusal, so a message can name the key a
    /// file SHOULD have written rather than merely the one it did.
    pub fn setpoint_key(self) -> &'static str {
        match self {
            MeasuredVariable::Level => "setpoint_m",
        }
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
/// **With one variant, "the setpoint's variable disagrees with the loop's" is
/// unrepresentable, and there is deliberately no guard for it.** A refusal path
/// nothing can reach is a coverage claim that cannot be checked
/// (`a-counter-is-not-a-gate`). The moment a second variant lands, writing a
/// setpoint through `Command::SetSetpoint` becomes able to change what a loop
/// measures, and that refusal becomes required rather than optional.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "variable", rename_all = "snake_case")]
pub enum ControlledValue {
    Level { m: Meter },
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
        }
    }

    /// `measurement − setpoint`, in this variable's SI unit.
    ///
    /// **The error's sign convention lives here and nowhere else.** Positive
    /// means "above setpoint", so a loop on a tank's OUTLET valve is direct
    /// acting: too much level opens the drain. A controller that computed its own
    /// difference would be free to disagree with the one a snapshot reader
    /// reconstructs from the two reported values, and the mutation this slice owes
    /// ("gain applied to the measurement instead of the error") only means
    /// anything while the error term is explicit and singly owned.
    ///
    /// Both arguments are the same type by construction, so a level measurement
    /// cannot be differenced against a pressure setpoint.
    pub fn error(measurement: Self, setpoint: Self) -> f64 {
        measurement.magnitude() - setpoint.magnitude()
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
    /// The node whose state is measured. A `Tank` for a level loop; the loader
    /// refuses anything else, because a level names nothing on a vessel whose
    /// state IS pressure.
    pub measurement_node: NodeId,
    /// The node this loop writes. A `Valve`; a `ReliefValve` is refused with its
    /// own reason, since its opening is actuated by its own inlet pressure.
    pub actuator: NodeId,
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
    /// debugging it. Real from load rather than optional: a level is a STORED
    /// quantity, so `Engine::new` seeds this by taking the measurement once, and
    /// a snapshot before the first tick reports a true level instead of a NaN or
    /// an absence.
    pub last_measurement: ControlledValue,
    /// The actuator position this loop last put on the faceplate, dimensionless
    /// in `[0, 1]`.
    ///
    /// In `Auto` it is the controller's output, which is also what was written to
    /// the valve. In `Manual` the loop writes nothing and this TRACKS the
    /// actuator's real opening, which is what a DCS faceplate shows and what makes
    /// AUTO→MANUAL transfer free (fork 4). Seeded from the valve's declared
    /// opening at load.
    pub last_output: f64,
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
    /// in the scenario file is both the id order and the execution order.
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

    /// Is `value` a legal setpoint for a loop measuring `node`?
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
    pub fn check_setpoint(&self, node: NodeId, value: ControlledValue) -> Result<(), SimError> {
        match (value, &self.node(node).kind) {
            (ControlledValue::Level { m }, NodeKind::Tank(t)) => {
                if !m.is_finite() || m.value() < 0.0 || m.value() > t.height.value() {
                    return Err(SimError::InvalidCommand(format!(
                        "level setpoint {} m is outside tank '{}''s range [0, {}] m — a                          setpoint the plant cannot reach leaves the loop pinned at                          saturation",
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
    /// A pressure or a temperature is SOLVED and lives in `last_solution` /
    /// `NodeStates`. A loop on either has no measurement at tick 0 and needs a
    /// stated rule for that tick — which is one more reason those variables are
    /// deferred per-variable rather than arriving "for free once the seam
    /// exists".
    ///
    /// Called at load to seed `ControlLoop::last_measurement` and once per loop
    /// per tick thereafter, so a loop's seeded measurement and its running one
    /// can never be taken by two different rules.
    ///
    /// # Errors
    /// `SimError::Scenario` if the node cannot answer for that variable — a level
    /// asked of anything that is not a `Tank`. The loader refuses that
    /// combination at load, so this is the rule-5 backstop for a hand-built
    /// graph, not the user-facing message.
    pub fn measure(
        &self,
        slate: &Slate,
        node: NodeId,
        variable: MeasuredVariable,
    ) -> Result<ControlledValue, SimError> {
        match (variable, &self.node(node).kind) {
            (MeasuredVariable::Level, NodeKind::Tank(t)) => Ok(ControlledValue::Level {
                m: t.level(slate),
            }),
            (MeasuredVariable::Level, _) => Err(SimError::Scenario(format!(
                "node '{}' is not a tank, so it has no level to control. A level names                  nothing on a vessel whose state IS pressure (docs/DESIGN.md §3a fork 2)",
                self.node(node).name
            ))),
        }
    }
}
