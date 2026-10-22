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
    Pump {
        h0: Meter,
        a: f64,
        on: bool,
        /// What cavitation costs this pump (M50, docs/DESIGN.md §55). Absent:
        /// the pump delivers its whole curve whatever its suction reads, and the
        /// wire is what it was before M50.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        suction: Option<PumpSuction>,
        /// Gas-locked (M54, docs/DESIGN.md §59.2): set by the ENGINE at the end
        /// of a tick in which this pump ran while its suction offered
        /// `GAS_LOCK_VOID_FRACTION` vapour by volume or more, and cleared only by
        /// `Command::VentPump` on the pump stopped. A locked pump makes no head
        /// whatever its suction offers — not a stop, not a trip, not the supply
        /// cooling clears it. Read by the solve as a fixed fact, like the relief
        /// latch (`Blowdown`). `false`, and absent from the wire, on every pump
        /// of a plant without a line flash.
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        gas_locked: bool,
        /// The gas collected in this pump's impeller eye, as a share of what
        /// locks it, in `[0, 1]` (M55.1, docs/DESIGN.md §60.1). Moved by the
        /// ENGINE at the end of each tick this pump runs: up by `dt` over its
        /// fill time while its suction offered `GAS_LOCK_VOID_FRACTION` vapour or
        /// more, down at the same rate while it offered less. Full is
        /// `gas_locked`, and a locked pump's pocket stays full until a vent
        /// empties both. Read by the solve as a fixed number: the pump delivers
        /// `(1 − gas_pocket)` of what its suction would let it. Absent from the
        /// wire while empty, so no pump that never filled one changes a byte.
        #[serde(default, skip_serializing_if = "is_empty_pocket")]
        gas_pocket: f64,
        /// The time an empty pocket takes to fill [s] while the suction offers
        /// `GAS_LOCK_VOID_FRACTION` vapour or more (M55.1): the scenario's
        /// `gas_fill_time_s`, absent for `GAS_POCKET_FILL_TIME`. Allowed only on a
        /// plant with a line flash, finite and positive (enforced at load).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        gas_fill_time: Option<Seconds>,
    },
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
    /// GIVEN UP, and stated rather than discovered: no chatter of its own, and —
    /// inherited from the gas valve's symmetry — it passes REVERSE flow, which a
    /// real one does not. Blowdown hysteresis (a real PSV recloses below its set
    /// pressure) is opt-in since M48 through `blowdown`: element state the solve
    /// reads and only the engine writes, between ticks (docs/DESIGN.md §53).
    ///
    /// The CURVE above still has no history. Which answer a plant with two
    /// answers takes does, since M48.1 and for every relief: the one where the
    /// valve stays as it stood at the last accepted solve, shut on the first
    /// (`network::solve_remembering_reliefs`, §53).
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
        /// A pop valve's memory (M48, docs/DESIGN.md §53). Absent: the memoryless
        /// M5 valve above, and the wire is what it was before M48.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        blowdown: Option<Blowdown>,
    },
    /// Check (non-return) valve: passes flow one way and shuts against the other
    /// (M30, docs/DESIGN.md §33, ledger row E22).
    ///
    /// The relief valve's sibling, and for the same reason a kind rather than a
    /// flag on `Valve`: its opening is not an operator setpoint but a memoryless
    /// function of the plant state. Where a PSV's spring reads its own inlet
    /// pressure, a check valve's disc reads the FORWARD DRIVE across its branch,
    /// `S = P_in − P_out − β` (the outlet pipe's static head folded in): shut at
    /// or below zero, fully open at `full_open`, and the relief valve's cubic
    /// smoothstep in between, so the characteristic stays C¹ where it meets both
    /// limits.
    ///
    /// **"Shut at `S ≤ 0`" is exact, not a threshold**: the branch's flow has the
    /// sign of `S`, so a shut disc is exactly a branch that would otherwise run
    /// backwards. Reverse flow is exactly zero, by `α = +∞`.
    ///
    /// **The disc reads the whole branch's drive, not its own drop**, because a
    /// device folds into its outlet pipe and the compiled edge sees only the
    /// total. The two are equal at zero flow — where the disc decides whether to
    /// open — and differ by the pipe's friction once it flows, which moves only
    /// how far into the band the disc sits.
    ///
    /// In gas service it takes `x_t` and folds through the same ISA compressible
    /// law a `Valve` does (M31, docs/DESIGN.md §34): the opening is read off the
    /// branch's drive first, and the valve it sets is then folded like any other.
    /// Opens from zero: a spring that holds the disc shut against a nonzero
    /// cracking pressure is ledger row E23.
    CheckValve {
        /// Coefficient at full lift, in `Valve::cv_max`'s SI form.
        cv_max: f64,
        /// Forward drive [Pa] at which the disc reaches full lift. Positive: a
        /// zero band is a step, which the Jacobian is not entitled to.
        full_open: Pascal,
        /// As `Valve::x_t` — required in gas service, refused in liquid.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        x_t: Option<f64>,
    },
    /// Zero-volume mixing point.
    Junction,
    /// Fired heater: a duty fired into a tube coil, and the coil heating the
    /// stream passing through it.
    ///
    /// Zero-volume in its FLUID like a pump or valve — the tube inventory is
    /// negligible against the throughput — but not in its METAL (M34,
    /// docs/DESIGN.md §37). The duty goes into the coil, a lumped body with a
    /// temperature of its own (`FurnaceCoil`), and the coil heats the fluid
    /// through a declared conductance. So the outlet is no longer
    /// `T_in + Q/(ṁ·cp)` on every tick: that is where it settles, and the coil's
    /// heat capacity is how long it takes to get there. With no flow the coil
    /// keeps what it absorbs, rather than the fluid's temperature growing as
    /// `1/ṁ` (DEFERRED B39), and it levels off at the flame (M36, §40).
    /// Hydraulically it is a plain pass-through at M2: the tube-side pressure
    /// drop belongs to the connecting pipes' resistance, not to a device
    /// characteristic.
    ///
    /// `duty` is the heat the burners RELEASE [W], a firing rate (M36,
    /// docs/DESIGN.md §40). The coil absorbs `Q·(T_f − T_c)/(T_f − T_a)` of it
    /// and the rest leaves up the stack, so the coil can never be heated past
    /// `flame_temperature`. Duty 0 is an unlit furnace; there is no separate
    /// `on` flag because there is nothing for one to express that 0 does not.
    ///
    /// Deliberately NOT stored in `Node::heat_input`: that field is the damage
    /// model's hook (fires), and a fire on a furnace must ADD to its duty, not
    /// overwrite the operator's setpoint. See `energy::heat_load`. Both reach
    /// the coil, not the fluid; only the duty passes through the flame law.
    Furnace {
        duty: Watt,
        coil: FurnaceCoil,
        /// Adiabatic flame temperature of the firing, `T_f` [K]: what the flue
        /// would reach keeping all the heat, and so the hottest the coil can
        /// get. Above the combustion air's `T_AMBIENT`.
        flame_temperature: Kelvin,
        /// The tubes' failure limit, the hole a failure opens, and whether they
        /// have failed (M37, docs/DESIGN.md §42).
        tubes: FurnaceTubes,
    },
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
    /// One cell of a furnace's coil ahead of the furnace node itself (M62.4c,
    /// docs/DESIGN.md §67.14): a zero-volume pass-through at its own pressure,
    /// holding its share of the coil's conductance against the furnace's one
    /// metal. Built by the loader from the furnace's `coil_cells`; the furnace
    /// node is the coil's last cell, its outlet.
    CoilCell {
        /// The furnace whose coil this cell is part of.
        furnace: NodeId,
    },
    /// A cooler on cooling water (M62.4b, docs/DESIGN.md §67.13): a
    /// counter-current exchanger between the stream through it and a
    /// cooling-water utility — liquid water at a constant heat capacity, not a
    /// plant stream — its duty a RESULT of the two inlets, the water's flow and
    /// its `UA`. Hydraulically a pass-through, like a `Cooler`; a separate unit
    /// so a duty cooler, its loops and its command keep their meaning.
    WaterCooler {
        /// Overall conductance times area, `UA` [W/K]. Positive and finite.
        ua: WattPerKelvin,
        /// The cooling water's inlet temperature [K]. Above its freezing point,
        /// below its boiling point.
        water_inlet: Kelvin,
        /// The cooling water's mass flow [kg/s], `Command::SetCoolingWaterFlow`.
        /// Non-negative; zero cools nothing.
        water_flow: KgPerSec,
    },
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
    /// Catalyst-bed reactor whose own heat sets its temperature (M57.1,
    /// docs/DESIGN.md §62): one feed in, one product out, through a lumped bed
    /// (`CatalystBed`) whose temperature is a STATE.
    ///
    /// Hydraulically a furnace — zero-volume fluid, 1-in-1-out, no pressure of
    /// its own. Thermally the furnace coil's twin (§37) with the flame replaced
    /// by the chemistry: the reaction runs at the bed's START-of-tick
    /// temperature over a residence time that follows the flow,
    /// `τ = V·ρ_feed/ṁ` (§62.2), its released heat goes into the bed, and the
    /// fluid leaves at the bed's temperature (`ε = 1`, §62.1).
    ///
    /// Unlike `Reactor`, nothing is held: a heat-releasing reaction can heat the
    /// bed into a runaway, and a heat-absorbing one cools it until it settles.
    /// The loader admits a bed only with a reaction model that is solved exactly
    /// at every residence time, `+∞` included (§62.2).
    ///
    /// Its `wall` fails past a metal limit, as a furnace's tubes burst (M58,
    /// docs/DESIGN.md §63): a hole on its outlet pipe, and a fire fed by what
    /// leaks, a declared share of which heats the bed.
    BedReactor { bed: CatalystBed, wall: BedWall },
    /// The hydrogen pass beside a catalyst bed (M59.1, docs/DESIGN.md §64.2):
    /// a hydrogen stream running through the bed it names, beside the bed's
    /// oil, sharing only the bed's temperature.
    ///
    /// Hydraulically a zero-volume gas node, 1-in-1-out, like a furnace: the
    /// pressure it solves to is the hydrogen pressure the bed's chemistry reads.
    /// Thermally the hydrogen leaves at the bed's temperature (`ε = 1`, §62.1's
    /// argument) and the heat it carries off is a term in the bed's first law;
    /// the heat sweep resolves the pass and its bed as ONE vertex, as it does a
    /// heat exchanger's two sides, so both inlets are known when the bed steps.
    ///
    /// Two streams, two pressures: the oil keeps its own line's pressure, a
    /// stated compromise (§64.2, the user's decision) — a real trickle bed has
    /// one.
    BedHydrogen {
        /// The `BedReactor` this pass runs through. Resolved from a name by the
        /// loader; at most one pass per bed.
        bed: NodeId,
        /// The bed's chemistry: every rate is multiplied by
        /// `f = (p/p_ref)^n` (§64.1), applied as the effective space time `f·τ`.
        kinetics: HydrogenKinetics,
        /// What the bed's reaction draws from this pass (M59.2, docs/DESIGN.md
        /// §64.11), `None` on a pass whose bed consumes nothing — every pass
        /// before M59.2, whose snapshot keeps its shape.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        consumption: Option<HydrogenConsumption>,
    },
    /// Centrifugal gas compressor at fixed speed (M59.1, docs/DESIGN.md §64.3):
    /// the hydrogen loop's recycle machine.
    ///
    /// A polytropic head falling with the SUCTION volume flow,
    /// `H_p(Q) = head − a·Q²` [J/kg], turned into a pressure rise at the suction
    /// density `ρ₁` — read at this node, its own inlet, whatever the flow's
    /// direction (§3a's higher-pressure-end convention is wrong for a machine) —
    /// and corrected for compressibility by the polytropic relation taken at the
    /// shutoff head (`network::compressor_multiplier`). Folded at its source
    /// into its single outlet edge, as a pump is. A stopped compressor makes no
    /// head. GAS ONLY, enforced at load. No heat of compression, surge or
    /// anti-surge (§64.3, deferred).
    Compressor {
        /// Polytropic head at zero flow [J/kg]. Positive.
        head: JPerKg,
        /// Head curve coefficient `a` [J/kg per (m³/s)²]: the head lost per
        /// square of suction volume flow. Non-negative.
        a: f64,
        /// Polytropic efficiency `η_p` (dimensionless), in (0, 1]:
        /// `(k−1)/k = (γ−1)/(γ·η_p)`.
        polytropic_efficiency: f64,
        on: bool,
    },
}

/// How a bed's chemistry reads its hydrogen (M59.1, docs/DESIGN.md §64.1):
/// every rate constant times `f = (p/p_ref)^n`, `p` the hydrogen pass's
/// pressure. Both numbers declared, no default; `p_ref` is the plant's design
/// hydrogen pressure, so `f = 1` there and the calibration holds.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HydrogenKinetics {
    /// The hydrogen order `n` (dimensionless), positive. A GAME VALUE (§64.1):
    /// ½, from a heavy-crude study seen only through its abstract's summary.
    pub order: f64,
    /// The pressure at which `f = 1` [Pa absolute]. Positive.
    pub reference_pressure: Pascal,
}

/// The pressure below which the hydrogen factor leaves its power law [Pa]:
/// 1.8 bar absolute (M59.4, docs/DESIGN.md §64.13; 1 bar from M59.2). A
/// NUMERICAL REGULARISATION, like the flow branches' 1 Pa linear zone, not a
/// fact about catalysts. Under 2 bar, the flare every shipped loop vents to,
/// so a loop that only vents never reaches it.
pub const HYDROGEN_FACTOR_RAMP_FROM: Pascal = Pascal(1.8e5);

/// The pressure at and below which nothing reacts [Pa]: 1.2 bar absolute, a
/// FLOOR above the atmosphere's 1.013 25 bar (M59.4, docs/DESIGN.md §64.13;
/// 1 kPa from M59.2). A loop starved of its makeup comes to REST here, holding
/// a stock far above the solve's tolerance, instead of being drained toward a
/// vacuum no solve can resolve — and, since M59.4, above the atmosphere, so a
/// hole in a starved loop still blows outward (a hole that draws air in is
/// refused, `network::finalize`). The same regularisation as
/// [`HYDROGEN_FACTOR_RAMP_FROM`].
pub const HYDROGEN_FACTOR_ZERO_AT: Pascal = Pascal(1.2e5);

impl HydrogenKinetics {
    /// `f = (p/p_ref)^n` at a pass pressure `pressure` [Pa] (dimensionless),
    /// regularised at low pressure (M59.2, docs/DESIGN.md §64.11; its floor
    /// raised above the atmosphere by M59.4, §64.13): from
    /// [`HYDROGEN_FACTOR_RAMP_FROM`] down it falls on a straight line to zero
    /// at [`HYDROGEN_FACTOR_ZERO_AT`], and is zero below.
    ///
    /// Why: at an order under one, the reaction's hydrogen demand over the
    /// loop's stock grows without bound as the pressure falls (`∝ p^(n−1)`),
    /// so a consuming loop with no makeup would drain itself faster each tick
    /// — measured, it drew a tick's stock twelve times over at 1e-6 Pa. On the
    /// ramp the demand falls with the stock and vanishes above the floor.
    /// Unchanged at and above 1.8 bar, so no plant whose loop stays there moves.
    pub fn factor(&self, pressure: Pascal) -> f64 {
        let p = pressure.value();
        let ramp_from = HYDROGEN_FACTOR_RAMP_FROM.value();
        if p >= ramp_from {
            return (p / self.reference_pressure.value()).powf(self.order);
        }
        let zero_at = HYDROGEN_FACTOR_ZERO_AT.value();
        if p <= zero_at {
            return 0.0;
        }
        let top = (ramp_from / self.reference_pressure.value()).powf(self.order);
        top * (p - zero_at) / (ramp_from - zero_at)
    }
}

/// The hydrogen a bed's reaction consumes from its pass (M59.2, docs/DESIGN.md
/// §64.11): the network's first nodal mass sink.
///
/// **The demand.** Each tick the bed's reaction draws
/// `D = ṁ_oil · Σᵢ (y_out,i − y_in,i) · uptakeᵢ` [kg/s] — per kilogram of each
/// lump formed, `uptake` kilograms of hydrogen, the feed's lumps the datum
/// (§64.6). The hydrogen's mass is already folded into the lumps' (§62.3), so
/// the oil keeps its mass and the gas side loses `D`.
///
/// **The sink.** The NEXT tick's solve draws `rate · P` at the pass, `P` the
/// pass's pressure that tick and `rate = D/Pⁿ` [kg/(s·Pa)] the demand over the
/// pressure its reaction ran at: implicit in the pressure, it vanishes as the
/// pass's pressure goes to zero, and it lags the reaction by one tick (§64.6's
/// three rules). What the solve drew — the pass's net inflow — is booked as
/// consumed, so the books close on the solve's own flows.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HydrogenConsumption {
    /// Hydrogen consumed per kilogram of each component FORMED [kg/kg], one per
    /// slate component in slate order, zero for an undeclared one. A GAME
    /// VALUE (§64.11). Non-negative, and never falling along any path the
    /// reaction runs, an undeclared lump counting as zero (§64.12), so the
    /// reaction consumes and never releases, whatever the bed is fed.
    pub uptake: Vec<f64>,
    /// The sink's rate this tick [kg/(s·Pa)], a STATE: the last tick's demand
    /// over the pressure its reaction ran at, written back at the end of every
    /// tick. Zero at load — the first tick has no reaction behind it.
    pub rate: f64,
}

/// A catalyst bed's vessel wall: its failure limit, the hole a failure opens,
/// the fire the leak feeds, and whether it has failed (M58, docs/DESIGN.md
/// §63) — `FurnaceTubes`' twin.
///
/// `Engine::run_burnouts` fails an intact wall whose bed stands at or past
/// `failure_temperature` at the top of a tick: it opens `hole` — the dormant
/// orifice the loader built on the bed's outlet pipe — to `rupture_area`, and
/// latches `state`. While the wall is FAILED, whatever leaks out of that hole
/// burns at `heating_value` per kilogram, and `fire_share` of that fire heats
/// the bed; the rest goes to the air. A player patches the hole with
/// `Command::PuncturePipe` at area zero, which puts the fire out with it, and
/// then repairs the wall (`Command::RepairBedWall`) once the bed is below its
/// limit.
///
/// The limit is compared with the LUMPED bed temperature (`CatalystBed`), as
/// an instant failure point, where a real shell fails by creep over time —
/// §42's simplifications for tubes, made again for a vessel (§63.2).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BedWall {
    /// Bed temperature at or above which the wall fails [K].
    pub failure_temperature: Kelvin,
    /// Area of the hole a failure opens [m²]. Positive.
    pub rupture_area: SquareMeter,
    /// Lower heating value of what leaks [J/kg]: what a kilogram of it
    /// releases burning outside the vessel. Non-negative.
    pub heating_value: JPerKg,
    /// The share of the leak's fire that heats the bed, in [0, 1]
    /// (dimensionless). A game value (§63.3): the rest goes to the air.
    pub fire_share: f64,
    /// The orifice edge the failure opens, on the bed's outlet pipe. Written by
    /// the loader once it has split that pipe, so `None` only on a graph built
    /// by hand without one — which the burn-out pass refuses rather than skips.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hole: Option<EdgeId>,
    /// The hydrogen side of the failure (M59.4, docs/DESIGN.md §64.7): a
    /// second hole, on the bed's hydrogen pass's outlet pipe, opened with the
    /// first and to the same area. `None` on a bed without a pass, and on one
    /// whose pass declares no heating value — every bed before M59.4, whose
    /// snapshot keeps its shape.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hydrogen_leak: Option<HydrogenLeak>,
    pub state: WallState,
}

/// The hydrogen a failed bed wall leaks (M59.4, docs/DESIGN.md §64.7): the
/// hole on the pass's outlet pipe and what a kilogram through it releases
/// burning. Its fire joins the oil's, and the wall's `fire_share` of the sum
/// heats the bed.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HydrogenLeak {
    /// Lower heating value of the hydrogen [J/kg]. Non-negative.
    pub heating_value: JPerKg,
    /// The orifice edge the failure opens, on the pass's outlet pipe. Written
    /// by the loader once it has split that pipe, as `BedWall::hole` is.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hole: Option<EdgeId>,
}

impl BedWall {
    /// Every hole a failure of this wall opens, the oil's first (M59.4): one
    /// on a bed without a hydrogen leak, two on one with.
    pub fn holes(&self) -> impl Iterator<Item = Option<EdgeId>> + '_ {
        std::iter::once(self.hole).chain(self.hydrogen_leak.as_ref().map(|leak| leak.hole))
    }

    /// A bed at `bed_temperature` [K] is at or past this wall's limit: the next
    /// burn-out pass fails it if it is intact. The one owner of the comparison,
    /// shared by the burn-out pass and `Command::RepairBedWall`.
    pub fn limit_reached(&self, bed_temperature: Kelvin) -> bool {
        bed_temperature >= self.failure_temperature
    }
}

/// Whether a catalyst bed's wall has failed (M58, docs/DESIGN.md §63).
///
/// Latched: `Failed` from the tick whose top pass found the bed at or past its
/// limit, until `Command::RepairBedWall`. Patching the hole does not clear it,
/// so a hole punched again by hand in a failed wall burns again.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum WallState {
    Intact,
    Failed { at_tick: u64 },
}

impl WallState {
    pub fn is_failed(self) -> bool {
        matches!(self, WallState::Failed { .. })
    }
}

/// A catalyst bed: the catalyst, the internals and the liquid they hold, as one
/// lumped body at one temperature (M57.1, docs/DESIGN.md §62.1).
///
/// No axial profile, no fluid–solid film, no pressure drop, no wall loss
/// (§62.1, rejected; §62.7, deferred). Its three numbers are declared per bed,
/// with no default, as a furnace coil's are.
///
/// `energy::bed_reactor` integrates its first law exactly across a tick, the
/// chemistry and the inflow frozen and the outflow linearised:
///
/// ```text
/// C·dT_b/dt = Q + H_in − H_out(T_b),   Q = −ṁ·Δh_rxn(T_b, τ)
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CatalystBed {
    /// Heat capacity of the bed, `C` [J/K]: catalyst, internals and the
    /// liquid they hold. Positive.
    pub heat_capacity: JPerK,
    /// Catalyst volume, `V` [m³]: the volume the space time `τ = V·ρ/ṁ` is
    /// quoted against (`1/LHSV`, §62.2). Positive.
    pub catalyst_volume: CubicMeter,
    /// Bed temperature, `T_b` [K]: a STATE, set from `bed_temperature_c` at
    /// load and written back at the end of every tick, so it is present before
    /// the first tick and reads the END of the last one — exactly
    /// `FurnaceCoil::temperature`.
    pub temperature: Kelvin,
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
    /// The draw's LINE when it runs through pumps and coolers to its store
    /// (M62.4b, docs/DESIGN.md §67.13): its pipes in order from the column,
    /// every one carrying the draw. Empty — and absent from the wire — on a
    /// draw whose `outlet` is its store, which keeps its single pipe.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub line: Vec<EdgeId>,
    /// The condenser's WATER draw (M62.5, docs/DESIGN.md §67.17): the free
    /// water decanted from the total condenser's drum, at its temperature. Its
    /// rate is a result of the column's water balance, so it declares no
    /// `draw_ratio`; it sits at `stage = 0`, listed second, after the
    /// distillate. False — and absent from the wire — on every other draw.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub water: bool,
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
            line: Vec::new(),
            water: false,
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
            line: Vec::new(),
            water: false,
        }
    }

    /// The condenser's water draw (M62.5): stage 0, no ratio.
    pub fn water_draw(outlet: NodeId) -> Self {
        Self {
            water: true,
            ..Self::by_stage(outlet, 0, None)
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
    /// Heat removed on stages between the top stage and the reboiler — the
    /// pumparounds of a crude column, modelled as side coolers (M60, docs/
    /// DESIGN.md §65.4). Only the heat-balance cascade reads them; empty on
    /// every other column, and then not serialized, which keeps every plant
    /// that predates them byte-identical.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pumparounds: Vec<Pumparound>,
    /// Steam into a stage by its own pipe (M62.6, docs/DESIGN.md §67.18), or
    /// `None`. Only the heat-balance cascade reads it; absent on every column
    /// that predates it, and then not serialized.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub steam: Option<SteamInlet>,
    /// The reboiler's duty held [W] (M64, docs/DESIGN.md §69), or `None`: the
    /// reboiler closes the column's energy balance, as it always has. Held, the
    /// bottom stage's balance is an equation and the reflux ratio the result —
    /// a steam-stripped column holds it at zero. Only the heat-balance cascade
    /// reads it; absent on every column that predates it, and then not
    /// serialized.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reboiler_duty: Option<Watt>,
    /// Pumparound LOOPS (M72, docs/DESIGN.md §77): liquid drawn off a stage,
    /// cooled and returned one stage up. Only the heat-balance cascade reads
    /// them; empty on every column that predates them, and then not
    /// serialized, which keeps every such plant byte-identical.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub loops: Vec<PumparoundLoop>,
    /// The column's trays, one geometry per section of stages (M73.1,
    /// docs/DESIGN.md §78.2): every stage but the reboiler in exactly one
    /// section. Only the heat-balance cascade reads them, to report each
    /// tray's hydraulics after its solve; empty on every column that predates
    /// them, and then not serialized, which keeps every such plant
    /// byte-identical.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub trays: Vec<TraySection>,
    /// The liquid the column holds (M74, docs/DESIGN.md §79.2), or `None`: the
    /// column is quasi-steady, as every column before M74. Only the
    /// heat-balance cascade reads it, on a column with trays; absent on every
    /// column that predates it, and then not serialized, which keeps every
    /// such plant byte-identical.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub holdup: Option<Box<ColumnHoldup>>,
}

/// One section of sieve trays on a heat-balance cascade (M73.1, docs/DESIGN.md
/// §78.2, §78.4.1): stages `from..=to`, one-based from the top, all built to
/// this geometry. The loader refuses each number outside the range its
/// correlations were checked over (`refinery_solvers::trays`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TraySection {
    /// The section's first stage, one-based from the top.
    pub from: u32,
    /// The section's last stage, `>= from`, above the reboiler.
    pub to: u32,
    /// The column's inside diameter [m].
    pub diameter: Meter,
    /// The distance from one tray to the next [m].
    pub spacing: Meter,
    /// The outlet weir's height [m].
    pub weir_height: Meter,
    /// A perforation's diameter [m]; the plate is taken as thick as its holes.
    pub hole_diameter: Meter,
    /// The holes' total area over the tray's active (bubbling) area [-].
    pub hole_area_share: f64,
    /// The tray's whole downcomer area over the column's [-] (Nutter's
    /// `DCA/A_t`); on `passes` passes each side downcomer holds this over the
    /// passes, and its chord is every pass's weir (docs/DESIGN.md §78.10).
    pub downcomer_share: f64,
    /// The gap under the downcomer's apron [m], below the weir so the
    /// downcomer stays sealed.
    pub clearance: Meter,
    /// The liquid's paths across the tray, 1 to 4.
    pub passes: u32,
}

/// The liquid a heat-balance cascade holds — on its trays, in its reflux drum
/// and in its base (M74, docs/DESIGN.md §79.2). Declared on a column with
/// trays, whose sizes give each tray's liquid; the drum's and the base's are
/// declared here as volumes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ColumnHoldup {
    /// The reflux drum's working volume [m³]; half full at its normal level.
    pub drum_volume: CubicMeter,
    /// The base's working volume [m³]; half full at its normal level.
    pub base_volume: CubicMeter,
    /// Whether the column starts with its reboiler held — the bottoms holding
    /// the base's level (§79.1's scheme B) — or free, the reboiler holding it
    /// (scheme A). The file's `reboiler = "held"` or `"free"`.
    pub start_held: bool,
    /// The tick the holdup advances by [s]: the plant's fixed timestep,
    /// copied from `[simulation] dt` by the loader (the timestep never changes
    /// after load).
    pub tick: Seconds,
    /// The liquid held: a STATE, `None` until the column's first tick fills it
    /// from its settled loads, then written back by the engine at the end of
    /// every tick that moves it (as a furnace's coil, §37).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub state: Option<HoldupState>,
}

/// What a column holds at the end of a tick (M74, docs/DESIGN.md §79.2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HoldupState {
    /// Each stage's liquid [kg], top stage first: a tray's on its deck and in
    /// its downcomer; the last, the reboiler's, the base's.
    pub stage_liquid: Vec<Kg>,
    /// Each stage's liquid composition, in the same order.
    pub stage_composition: Vec<Composition>,
    /// The reflux drum's liquid [kg].
    pub drum_liquid: Kg,
    /// The reflux drum's liquid composition: the distillate's and the
    /// reflux's.
    pub drum_composition: Composition,
    /// Each stage's temperature [K], in the same order.
    pub stage_temperature: Vec<Kelvin>,
    /// The drum's temperature [K]: its liquid's bubble point.
    pub drum_temperature: Kelvin,
    /// The vapour rising into each tray from the stage below [kg/s], top
    /// tray first — what the tray's liquid is read against next tick (its
    /// dry-plate head in the downcomer's backup).
    pub tray_vapour: Vec<KgPerSec>,
    /// That vapour's density [kg/m³], in the same order.
    pub tray_vapour_density: Vec<KgPerM3>,
    /// The pressure drop each tray fed back to the stage below this tick [Pa],
    /// top tray first: lagged behind what the trays' loads ask by the vapour
    /// the column holds between its trays.
    pub tray_drops: Vec<Pascal>,
    /// The flows the first tick settled on, which the level loops' outputs
    /// are multiples of: the distillate [kg/s], the bottoms [kg/s] and the
    /// reboiler's duty [W].
    pub settled_distillate: KgPerSec,
    pub settled_bottoms: KgPerSec,
    pub settled_reboiler: Watt,
    /// The drum's and the base's level loops' integral terms [-], each a
    /// share of its settled output.
    pub drum_integral: f64,
    pub base_integral: f64,
    /// Whether the reboiler was held this tick (scheme B, the bottoms on the
    /// base's level) or free (A), and the bottoms [kg/s] and reboiler's duty
    /// [W] it ran at: a tick that finds the hold changed hands the base's
    /// level to the other stream from where that stream stood.
    pub held: bool,
    pub last_bottoms: KgPerSec,
    pub last_reboiler: Watt,
}

/// A pumparound loop on a heat-balance cascade (M72, docs/DESIGN.md §77.1):
/// liquid drawn off `draw_stage` at `flow`, cooled to `return_temperature` and
/// returned to `return_stage`, one stage up. Its duty is a RESULT — the heat
/// the cooling takes out of the circulating liquid — and the trays between the
/// two stages carry the circulation on top of their own liquid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PumparoundLoop {
    /// The stage it draws from, one-based from the top, strictly between the
    /// top stage and the reboiler.
    pub draw_stage: u32,
    /// The stage it returns to: `draw_stage − 1`, the only one admitted (the
    /// stage balances stay tridiagonal, §77.1).
    pub return_stage: u32,
    /// The circulation [kg/s], ≥ 0; zero is a loop that is not running.
    pub flow: KgPerSec,
    /// The temperature its cooler returns it at [K].
    pub return_temperature: Kelvin,
    /// The most its cooler can take out [W], > 0: past it the return warms
    /// (§77.3).
    pub cooler_capacity: Watt,
    /// The duty a cold start brings the cooler in at [W], ≥ 0, before the
    /// held return temperature takes over (§77.5) — a game value.
    pub start_duty: Watt,
    /// The loop's LINE on the map (M72.2, docs/DESIGN.md §77.2): its pipes in
    /// order from the column, through its pumps and its one cooler, the last
    /// pipe back INTO the column — every one carrying the loop's flow,
    /// prescribed, as a draw line's carry its draw. Written by the loader;
    /// empty on a loop built by hand without one, and then not serialized.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub line: Vec<EdgeId>,
    /// The cooler on the line, whose duty the column writes each tick — the
    /// heat that brings the loop to its return temperature. `None` exactly
    /// when `line` is empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cooler: Option<NodeId>,
}

/// A steam inlet on a cascade column (M62.6, docs/DESIGN.md §67.18): the pipe
/// its water arrives by and the stage it enters. The steam's RATE is the pipe's
/// flow — a result of the supply's pressure and its valve, as on a real unit —
/// and its water joins the stage's water balance, its enthalpy the stage's
/// energy balance.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SteamInlet {
    /// The stage it enters, one-based from the top, in `1..=stages`.
    pub stage: u32,
    /// The edge INTO the column the steam arrives on — the column's second
    /// inflow, never mixed into its feed.
    pub edge: EdgeId,
}

/// One pumparound on a cascade column (M60, docs/DESIGN.md §65.4): heat taken
/// out of one stage. A real pumparound draws liquid off a stage, cools it and
/// returns it higher up; what the stage balances see of it is the heat, and
/// that is all this models — the liquid's circuit between two stages is not.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pumparound {
    /// The stage it cools, one-based from the top, strictly between the top
    /// stage and the reboiler.
    pub stage: u32,
    /// Heat removed [W], a non-negative magnitude (the cooler convention).
    pub duty: Watt,
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

/// A furnace's tubes as something that can FAIL (M37, docs/DESIGN.md §42): the
/// burn-out, damage the engine raises from a state rather than a command.
///
/// Past `failure_temperature` the coil bursts: on the next tick's top pass the
/// engine opens `hole` — a leak orifice the loader built on the furnace's outlet
/// pipe, dormant until then — to `rupture_area`, and latches `state`. While the
/// tubes are FAILED, whatever leaks out of that hole burns in the firebox, at
/// `heating_value` per kilogram, as extra FUEL (it passes through the flame law
/// like the duty). A player patches the hole with `Command::PuncturePipe` at area
/// zero, which puts the fire out with it, and then replaces the tubes
/// (`Command::ReplaceTubes`), which re-arms them.
///
/// The limit is compared with the LUMPED coil temperature (`FurnaceCoil`), not a
/// peak tube-skin temperature, and as an instant failure point, where a real
/// tube's life at temperature is a creep-rupture time. Both are the model's
/// simplifications, said in §42.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FurnaceTubes {
    /// Coil temperature at or above which the tubes fail [K].
    pub failure_temperature: Kelvin,
    /// Area of the hole a failure opens [m²]. Positive.
    pub rupture_area: SquareMeter,
    /// Lower heating value of the fluid the furnace heats [J/kg]: what a
    /// kilogram of it releases burning in the firebox. Zero for a fluid that
    /// does not burn (water). Non-negative.
    pub heating_value: JPerKg,
    /// The orifice edge the burn-out opens, on the furnace's outlet pipe.
    /// Written by the loader once it has split that pipe, so `None` only on a
    /// graph built by hand without one — which the burn-out pass refuses rather
    /// than skips (rule 5: a furnace that can fail must have somewhere to leak).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hole: Option<EdgeId>,
    pub state: TubeState,
}

impl FurnaceTubes {
    /// A coil at `coil_temperature` [K] is at or past these tubes' limit: the
    /// next burn-out pass bursts them if they are intact. The one owner of the
    /// comparison, shared by the burn-out pass, `Command::ReplaceTubes` and a
    /// trip's restart.
    pub fn limit_reached(&self, coil_temperature: Kelvin) -> bool {
        coil_temperature >= self.failure_temperature
    }

    /// Burst, or bursting on the next burn-out pass: tubes no trip may relight
    /// (M41, docs/DESIGN.md §46). The second half is reachable on a furnace a
    /// trip holds dark, because a fire (`Command::SetHeatInput`) still heats its
    /// coil, and the trips run before the burn-outs in a tick.
    pub fn burst_or_bursting(&self, coil_temperature: Kelvin) -> bool {
        self.state.is_failed() || self.limit_reached(coil_temperature)
    }
}

/// Whether a furnace's tubes have burnt out (M37, docs/DESIGN.md §42).
///
/// Latched: `Failed` from the tick whose top pass found the coil at or past its
/// limit, until `Command::ReplaceTubes`. Patching the hole does not clear it, so
/// a hole punched again by hand on failed tubes burns again.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum TubeState {
    Intact,
    Failed { at_tick: u64 },
}

impl TubeState {
    pub fn is_failed(self) -> bool {
        matches!(self, TubeState::Failed { .. })
    }
}

/// A furnace's tube coil: the metal the duty heats, and through which the duty
/// reaches the fluid (M34, docs/DESIGN.md §37).
///
/// One lumped body at one temperature — no radiant and convective sections, no
/// wall-thickness gradient, no tube-to-tube spread. Its two parameters are
/// declared per furnace, with no default: nothing in the engine derives a
/// coil's mass or its film coefficient from a duty.
///
/// **The fluid sees a tube wall at uniform temperature `T_c`**, so its outlet is
/// the constant-surface-temperature result for flow in a tube (Incropera &
/// DeWitt, *Fundamentals of Heat and Mass Transfer*, 6th ed., eq. 8.42b):
///
/// ```text
/// T_out = T_c − (T_c − T_in)·exp(−UA/(ṁ·cp))
/// ```
///
/// and the metal is a lumped capacitance (ibid. §5.1) holding the difference:
///
/// ```text
/// C·dT_c/dt = Q − G·(T_c − T_in),   G = ṁ·cp·(1 − exp(−UA/(ṁ·cp)))
/// ```
///
/// `energy::furnace_coil` integrates that exactly across a tick (it is linear
/// in `T_c` with everything else frozen), so no coil is too light for `dt`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FurnaceCoil {
    /// Heat capacity of the tube metal, `C` [J/K]. Positive.
    pub heat_capacity: JPerK,
    /// Metal-to-process conductance, `UA` [W/K]: the inside film coefficient
    /// times the wetted area. Positive — a coil that passes no heat heats
    /// nothing. Constant: it does not fall with the flow, which a real film
    /// coefficient does (`h ∝ Re^0.8`).
    pub conductance: WattPerKelvin,
    /// Metal temperature, `T_c` [K]: a STATE, set from `coil_temperature_c` at
    /// load and written back at the end of every tick, so it is present before
    /// the first tick and reads the END of the last one.
    pub temperature: Kelvin,
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

/// A pop relief valve's memory (M48, docs/DESIGN.md §53, ledger row B6). See
/// `NodeKind::ReliefValve::blowdown`.
///
/// Shut, the valve runs the M5 curve unchanged. Lifted, it runs at FULL LIFT,
/// whatever its inlet reads, until the inlet falls below `set − amount` — the
/// reseat pressure (API 520 Part I's blowdown) — which is the hysteresis: at one
/// pressure between reseat and set, the valve is shut on the way up and open on
/// the way down.
///
/// `lifted` is written by the engine ONLY, at the top of each tick from the
/// pressure its spring sensed in the last solve (`Engine::run_relief_latches`),
/// and read by the solve; nothing inside a solve moves it.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Blowdown {
    /// How far below the set pressure [Pa] a lifted valve reseats. Positive, and
    /// less than the set pressure, both enforced at load.
    pub amount: Pascal,
    /// Lifted since its inlet last stood above the set, and not yet reseated.
    pub lifted: bool,
}

impl Blowdown {
    /// The latch after a solve whose inlet pressure [Pa absolute] was
    /// `inlet_pressure`, for a valve set at `set_pressure` [Pa absolute]: shut
    /// lifts strictly above set, lifted reseats strictly below `set − amount`,
    /// and in between it stays as it was.
    pub fn after(self, inlet_pressure: Pascal, set_pressure: Pascal) -> Blowdown {
        let p = inlet_pressure.value();
        let lifted = if self.lifted {
            p >= set_pressure.value() - self.amount.value()
        } else {
            p > set_pressure.value()
        };
        Blowdown { lifted, ..self }
    }
}

/// The vapour's share of a running pump's suction VOLUME [-] at which it locks
/// (M54, docs/DESIGN.md §59.2; `NodeKind::Pump::gas_locked`): RELAP5's fully
/// degraded point, where its two-phase head multiplier reaches 1 (RELAP5/MOD3
/// Code Manual Vol. I, NUREG/CR-5535-V1, Table 3.5-3). The table itself is the
/// solver's (`refinery_solvers::elements::two_phase_head_multiplier`), which a
/// reference test holds to this number.
pub const GAS_LOCK_VOID_FRACTION: f64 = 0.165;

/// The time an empty gas pocket takes to fill [s] — and so to lock a pump —
/// while its suction offers `GAS_LOCK_VOID_FRACTION` vapour or more, where a
/// plant does not set it (M55.1, docs/DESIGN.md §60.1; `NodeKind::Pump::
/// gas_pocket`). Not a published number: none was found for how long a
/// centrifugal pump runs on a gassy suction before it locks. Chosen by the user
/// as game feel — long enough that the one tick a step can mislead (B50) or a
/// brief surge cannot lock a pump, short enough that a pump past its lock point
/// visibly locks.
pub const GAS_POCKET_FILL_TIME: Seconds = Seconds(3.0);

/// Whether a pump's gas pocket is empty — what keeps it off the wire (M55.1).
fn is_empty_pocket(pocket: &f64) -> bool {
    *pocket == 0.0
}

/// A pump's suction limit (M50, docs/DESIGN.md §55, ledger row B9). See
/// `NodeKind::Pump::suction`.
///
/// The pump delivers `φ·H(Q)`, where `φ` falls from 1 to 0 as the net positive
/// suction head available, `NPSHa = (P_suction − P_bubble)/(ρ·g)`, falls from
/// well above `npsh_required` to zero. The curve is the solver's
/// (`elements::cavitation_head_fraction`); this struct holds what it reads.
///
/// `bubble_pressure` is written by the engine ONLY, at the top of each tick,
/// from the cavitation criterion of the last tick (the pump's own liquid at its
/// own resolved temperature), and read by the solve as a fixed number — the
/// relief latch's arrangement (`Blowdown`). `None` before the first tick and
/// whenever the last tick had no criterion at this pump (a gas): the pump then
/// delivers its whole curve.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PumpSuction {
    /// Net positive suction head required [m]: the `NPSHa` at which the pump's
    /// head has fallen 3% (ANSI/HI 9.6.1's NPSH3). Positive, enforced at load.
    pub npsh_required: Meter,
    /// Bubble pressure [Pa] of the liquid at this pump, as the last tick left
    /// it. The solve this tick reads it; the engine moves it between ticks.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bubble_pressure: Option<Pascal>,
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

    /// The free water standing at this tank's bottom: its slate position and
    /// its mass [kg], or `None` on a slate with no free-water member or while
    /// the tank holds none (M62.3, docs/DESIGN.md §67.3).
    ///
    /// Free water never dissolves in the oil (§67.2), so whatever water a tank
    /// holds is a layer under the oil. The inventory is still one composition
    /// as STATE; the layers are what its outlet and its brim read
    /// (`drawn_composition`, `top_spill`). `level`, `bottom_pressure` and
    /// `capacity` need nothing from this: ideal mixing makes the mixture's
    /// volume the sum of the layers' volumes, and the floor carries `m·g/A`
    /// however the mass is stacked.
    pub fn free_water(&self, slate: &Slate) -> Option<(usize, Kg)> {
        let water = slate.free_water()?;
        let fraction = self.composition.fractions()[water];
        (fraction > 0.0).then(|| (water, Kg(fraction * self.mass.value())))
    }

    /// The height of the water layer [m], `m_w/(ρ_w·A)`, on every tank of a
    /// slate with a free-water member — zero while it holds none — and `None`
    /// on any other slate (M62.3, docs/DESIGN.md §67.3).
    pub fn water_level(&self, slate: &Slate) -> Option<Meter> {
        let water = slate.free_water()?;
        let mass = self.composition.fractions()[water] * self.mass.value();
        let density = slate.get(water).density.map_or(f64::NAN, |d| d.value());
        Some(Meter(mass / (density * self.area.value())))
    }

    /// What this tank's outlets carry while `drawn` [kg] leaves through them
    /// over one tick: **the bottom layer first** (M62.3, docs/DESIGN.md §67.3,
    /// decision 9). Free water while any stands, then the oil at its own
    /// fractions; on the tick the water runs out, all of it and the oil after
    /// it, so no outlet can take water the tank does not hold. `None` where no
    /// water stands: the outlets carry the inventory itself, as they always
    /// have.
    ///
    /// One answer for every outlet, from their TOTAL draw: two outlets sizing
    /// their water share each from its own flow would each claim the whole
    /// layer. `drawn` beyond the inventory is read as the whole inventory (a
    /// tank drawn past empty is starved and swept as a mixing point instead).
    ///
    /// # Errors
    /// `SimError::Numerical` if the weights form no valid composition.
    pub fn drawn_composition(
        &self,
        slate: &Slate,
        drawn: Kg,
    ) -> Result<Option<Composition>, SimError> {
        let Some((water, standing)) = self.free_water(slate) else {
            return Ok(None);
        };
        let drawn = drawn.value().min(self.mass.value());
        if drawn <= standing.value() {
            return Ok(Some(Composition::pure(slate.len(), water)));
        }
        // All the water, and the oil's share `(drawn − m_w)/(m − m_w)` of each
        // oil component: positive, because `drawn > m_w` and `drawn ≤ m`.
        let oil_share = (drawn - standing.value()) / (self.mass.value() - standing.value());
        let weights: Vec<f64> = self
            .composition
            .fractions()
            .iter()
            .enumerate()
            .map(|(c, fraction)| {
                if c == water {
                    standing.value()
                } else {
                    fraction * self.mass.value() * oil_share
                }
            })
            .collect();
        Composition::from_weights(&weights).map(Some)
    }

    /// What spills over the brim of a tank holding free water, as a mass [kg]
    /// and a composition, or `None` while it stands at or below its brim or
    /// holds no water (M62.3, docs/DESIGN.md §67.3; the user chose it while building).
    ///
    /// **The top layer first**: the oil, at its own fractions, until the
    /// liquid's volume `m/ρ(x)` is back at `A·H`; the water only once the oil
    /// is gone. A volume, not a mass, is what the brim measures, and the oil's
    /// is closed-form under ideal mixing: the excess volume over `A·H` is
    /// spilled at the oil's own density `ρ_oil`. The dry overflow keeps its
    /// own rule on mass (`capacity`), which this does not touch.
    ///
    /// # Errors
    /// `SimError::Numerical` if the weights form no valid composition.
    pub fn top_spill(&self, slate: &Slate) -> Result<Option<(Kg, Composition)>, SimError> {
        let Some((water, standing)) = self.free_water(slate) else {
            return Ok(None);
        };
        let capacity = self.capacity(slate).value();
        if self.mass.value() <= capacity {
            return Ok(None);
        }
        let excess_volume = (self.mass.value() - capacity) / self.density(slate).value();
        let oil_mass = self.mass.value() - standing.value();
        // The oil's own fractions and density: the inventory with its water
        // taken out (`1/ρ_oil = Σ_oil w'/ρ`, the mixture's own rule).
        let oil_weights: Vec<f64> = self
            .composition
            .fractions()
            .iter()
            .enumerate()
            .map(|(c, fraction)| if c == water { 0.0 } else { *fraction })
            .collect();
        let spilled_weights: Vec<f64> = if oil_mass > 0.0 {
            let oil = Composition::from_weights(&oil_weights)?;
            let oil_volume = oil_mass / oil.mixture_density(slate).value();
            if oil_volume >= excess_volume {
                let spilled_oil = excess_volume * oil.mixture_density(slate).value();
                oil.fractions().iter().map(|f| f * spilled_oil).collect()
            } else {
                // Every drop of oil, and the water above the brim after it.
                let water_density = slate.get(water).density.map_or(f64::NAN, |d| d.value());
                let spilled_water = (excess_volume - oil_volume) * water_density;
                oil.fractions()
                    .iter()
                    .enumerate()
                    .map(|(c, f)| {
                        if c == water {
                            spilled_water.min(standing.value())
                        } else {
                            f * oil_mass
                        }
                    })
                    .collect()
            }
        } else {
            let water_density = slate.get(water).density.map_or(f64::NAN, |d| d.value());
            let mut weights = vec![0.0; slate.len()];
            weights[water] = (excess_volume * water_density).min(standing.value());
            weights
        };
        let spilled: f64 = spilled_weights.iter().sum();
        Ok(Some((
            Kg(spilled),
            Composition::from_weights(&spilled_weights)?,
        )))
    }

    /// Spill the top layer (`top_spill`) out of this tank: debit it component
    /// by component and leave the tank exactly AT its brim, returning what
    /// left, a mass [kg] and a composition. `None`, and nothing touched, where
    /// `top_spill` has nothing to spill.
    ///
    /// **The brim is a tie, as on the dry overflow** (M23): the spill moves the
    /// composition and so the capacity, and `held − spill` lands within a few
    /// ULP of the new `ρ(x')·A·H` — above it about half the time, which would
    /// spill a rounding error on every tick a full wet tank stands still. So
    /// the mass is set to the new capacity and the spill reported as the
    /// rest, at the spilled composition: the books move by ULPs.
    ///
    /// # Errors
    /// `SimError::Numerical` if the remainder forms no valid composition.
    pub fn spill_top(&mut self, slate: &Slate) -> Result<Option<(Kg, Composition)>, SimError> {
        let Some((spill, over)) = self.top_spill(slate)? else {
            return Ok(None);
        };
        let held = self.mass.value();
        let weights: Vec<f64> = self
            .composition
            .fractions()
            .iter()
            .zip(over.fractions())
            .map(|(x, y)| (x * held - y * spill.value()).max(0.0))
            .collect();
        self.composition = Composition::from_weights(&weights)?;
        self.mass = Kg(self.capacity(slate).value().min(held));
        Ok(Some((Kg(held - self.mass.value()), over)))
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
///
/// **A `Coil` is a furnace's tube metal** (M35, docs/DESIGN.md §39), read for a
/// temperature by a TRIP only: the tube-skin temperature a real heater trips on.
/// It is a point of its own rather than a second variable on the furnace node,
/// because the node's `temperature` is its OUTLET and a variable has one meaning
/// per point. A loop on a coil is refused by the loader, by name.
///
/// **Serialized as the file writes it, less the variable** (M40,
/// docs/DESIGN.md §45): `{"node":3}`, `{"pipe":5}` or `{"coil":2}`, the id in
/// place of the name. `TripSnapshot::watches` publishes it, and the variable is
/// already the tag of the trip's `limit`, so a second copy would be a second
/// owner of one fact.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MeasurementPoint {
    Node(NodeId),
    Pipe(EdgeId),
    Coil(NodeId),
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
/// furnace reverse, a valve holding its own flow reverse). On a level or
/// pressure loop a valve's sign runs through the holdup and is TOPOLOGY: since
/// M29 the loader reads it in one hop — a drain is direct, a fill must say
/// reverse — and refuses a valve that is neither (docs/DESIGN.md §32). Both
/// checks are one hop because a valve has exactly one inlet and one outlet
/// (docs/DESIGN.md §24 fork 3).
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

/// What a loop does while the pump feeding its valve is stopped (M45.1,
/// docs/DESIGN.md §50; `docs/DEFERRED.md` E25): in AUTO, with `pump` off, it
/// writes `output` to its valve every tick and re-seeds its memory against it
/// — output tracking — so a restart resumes from `output` instead of from a
/// valve it wound wide open while nothing came through.
///
/// Declared by the file, pump and position both: which pump a valve depends on
/// is not something the loader may infer from the pipes (the E21 lesson), and a
/// held position is the file's to say, as a trip's `SetValve` is.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OnPumpStop {
    /// The pump whose stop the loop holds for.
    pub pump: NodeId,
    /// The valve opening it holds, in `[0, 1]`.
    pub output: f64,
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
    /// The limit this loop's actuator is SATURATED at, for a cascade primary's
    /// hold (M28's rule, latched by E19's fix, docs/DESIGN.md §38). `None` at
    /// load and whenever the loop is not acting.
    ///
    /// Set on any tick the loop's position is exactly at a limit, which is
    /// M28's whole test. **Cleared only when the loop's own measurement reaches
    /// its setpoint** — when the error that drove it to the limit changes sign —
    /// or when it stops acting, or reaches the other limit. Not on the tick its
    /// output merely dips off the limit: on a lagging plant a pinned PI loop's
    /// back-calculated memory is one tick behind a measurement still creeping
    /// toward the target, so its output lands a hair inside the limit for a few
    /// ticks at a time while the actuator's authority is still spent. Engine
    /// state; never on the wire.
    pub saturated: Option<ActuatorLimit>,
    /// What the loop does while the pump feeding its valve is stopped (M45.1,
    /// docs/DESIGN.md §50). `None` on every loop that does not declare it, which
    /// then runs exactly as it did before M45.
    pub on_pump_stop: Option<OnPumpStop>,
    /// The quench check a loop on a catalyst bed carries (M57.4, docs/DESIGN.md
    /// §62.11): `Some` exactly when the loop measures a bed — a single loop on a
    /// quench valve, or a cascade primary over a flow loop on one — and `None`
    /// on every other loop, which then runs exactly as it did before M57.4.
    pub quench: Option<QuenchCheck>,
    /// The two temperatures the last quench check compared: at load, then at
    /// the top of every tick. `Some` exactly when `quench` is.
    pub last_quench: Option<QuenchReading>,
}

/// Where a quench loop's sign comes from (M57.4, docs/DESIGN.md §62.11). A loop
/// holding a catalyst bed's temperature by mixing a cold stream into its feed
/// acts the right way only while that stream is colder than the one it joins,
/// which no walk of the pipes can see. Found by the loader's `quench_layout`;
/// read by [`PlantGraph::quench_reading`] at load and at the top of every tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuenchCheck {
    /// The bed the loop measures; the mix's single outlet line feeds it, or
    /// its hydrogen pass (a hydrogen quench, M59.3).
    pub bed: NodeId,
    /// The pipe that brings the quench into the mix.
    pub quench_pipe: EdgeId,
    /// The mix's other inlet: the MAIN stream the quench joins.
    pub main_pipe: EdgeId,
    /// Where the quench line starts, walked upstream from its valve: a `Tank`,
    /// a `Source` or — for a hydrogen quench (M59.3, docs/DESIGN.md §64.12) —
    /// a gas `Vessel`, whose temperature is a state present from load.
    pub origin: NodeId,
}

/// The two temperatures one quench check compared (M57.4, docs/DESIGN.md
/// §62.11).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct QuenchReading {
    /// The quench arriving at the mix [K]: its pipe's stream when the last solve
    /// gave that pipe a forward flow, else its origin's own temperature.
    pub quench_k: Kelvin,
    /// The main stream it joins [K]: its pipe's stream when the last solve gave
    /// that pipe a forward flow, else the measured bed's own temperature.
    pub joins_k: Kelvin,
}

impl QuenchReading {
    /// Whether the quench is colder than what it joins — STRICTLY: the only
    /// state in which more of it is sure to lower the mix's temperature.
    pub fn is_cold(&self) -> bool {
        self.quench_k.value() < self.joins_k.value()
    }
}

/// Which end of its range a loop's actuator is saturated at (docs/DESIGN.md
/// §38): `Top` at output 1, `Bottom` at output 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorLimit {
    Top,
    Bottom,
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
/// `state` key nested in a `state` field. A trip pressed by hand adds
/// `"by_hand":true` (M38, docs/DESIGN.md §43); a trip its measurement fired
/// writes the M22 form unchanged.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum TripState {
    Armed,
    /// Latched. `at_tick` is the first tick that runs in the safe state — for a
    /// trip its measurement fired, the tick whose trip pass fired it, the number
    /// a snapshot of that tick carries — so a frontend sampling every tenth
    /// snapshot still knows exactly when. A reset returns the trip to `Armed`
    /// and the tick goes with it: this is "tripped now, since when", not "last
    /// tripped at" (a trip's history is `docs/DEFERRED.md` E15).
    ///
    /// **A press lands between ticks** (`Command::ManualTrip`, M38), so its
    /// `at_tick` is the tick AFTER the snapshot standing when it was pressed:
    /// the next tick is the first to run in the safe state, exactly as for a
    /// trip that fires on that tick's pass.
    Tripped {
        at_tick: u64,
        /// Pressed by hand rather than fired by the measurement. Skipped on the
        /// wire when false, so every trip a measurement fires serializes as it
        /// did before M38.
        #[serde(default, skip_serializing_if = "is_false")]
        by_hand: bool,
    },
}

fn is_false(value: &bool) -> bool {
    !*value
}

impl TripState {
    pub fn is_tripped(self) -> bool {
        matches!(self, TripState::Tripped { .. })
    }
}

/// How a latched trip is reset, and whether its reset hands back what it
/// stopped (M40, docs/DESIGN.md §45; `docs/DEFERRED.md` E15).
///
/// **`Manual` is the default and the real-plant rule** (IEC 61511 asks for a
/// person to reset a safety function): a person resets it and the reset
/// restarts nothing, which is every trip before M40. The other two are opt-in
/// per trip, in the file.
///
/// **A restart hands the equipment back as it stood before the trip**: a loop
/// that was in AUTO on it goes back to AUTO through the bumpless transfer, from
/// where the equipment stands (so a relit furnace ramps up from zero rather than
/// jumping to its old firing); equipment with no such loop goes back to its
/// pre-trip state — a pump on, a valve at its opening, a furnace at its duty.
/// **It happens only when every trip that held that equipment during the stop
/// allows it, and none was pressed by hand**: a trip whose reset is `Manual`,
/// or an emergency stop, keeps it stopped for a person to restart.
///
/// Tagged `mode`, skipped on the wire when `Manual`, so every trip written
/// before M40 publishes the bytes it did.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum TripReset {
    /// A person resets it (`Command::ResetTrip`); the reset restarts nothing.
    #[default]
    Manual,
    /// A person resets it, and the reset restarts the equipment.
    ManualRestart,
    /// The trip resets ITSELF on the trip pass whose measurement is past
    /// `reset_at` on the safe side — strictly below it for a high trip, above
    /// it for a low one — and restarts the equipment. A person may still reset
    /// it sooner, under `ResetTrip`'s rule, with the same restart.
    ///
    /// **`reset_at` is a deadband, required and strictly on the safe side of
    /// the limit** (the loader refuses equal): a trip re-armed at its own limit
    /// would cut and relight on alternate ticks. A trip pressed by hand never
    /// resets itself.
    Auto { reset_at: ControlledValue },
}

impl TripReset {
    pub fn is_manual(&self) -> bool {
        matches!(self, TripReset::Manual)
    }

    /// Whether a reset of this trip may hand its equipment back.
    pub fn restarts(self) -> bool {
        !self.is_manual()
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
    /// Cut a furnace's fuel: its safe state is zero duty (M32,
    /// docs/DESIGN.md §35). One safe state, so no position: a furnace held at
    /// some lower firing is not a cut, and two trips on one furnace cannot
    /// disagree about it. The stream still flows through, unheated.
    CutFurnace { furnace: NodeId },
}

impl TripAction {
    /// The node this action writes.
    pub fn equipment(self) -> NodeId {
        match self {
            TripAction::StopPump { pump } => pump,
            TripAction::SetValve { valve, .. } => valve,
            TripAction::CutFurnace { furnace } => furnace,
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
    /// Where the trip measures. A `Node` for a tank's level, a vessel's pressure
    /// or a holdup's temperature, which are stored and exist from load (fork 2);
    /// a declared `Pipe` for a flow (M33, docs/DESIGN.md §36), which is absent
    /// for exactly one trip pass, tick 1's, before any solve — the trip stays
    /// armed and compares nothing on that pass. A furnace's or cooler's outlet,
    /// absent again whenever the unit stagnates, is refused
    /// (`docs/DEFERRED.md` E13).
    pub measurement_point: MeasurementPoint,
    pub direction: TripDirection,
    /// The limit, and — through `ControlledValue::variable` — what is measured.
    pub limit: ControlledValue,
    /// Non-empty; each names one pump, valve or furnace and its safe state.
    pub actions: Vec<TripAction>,
    /// Who resets it, and whether the reset restarts the equipment (M40).
    pub reset: TripReset,
    /// Other trips that must be clear before this one's reset may hand its
    /// equipment back — the start permissives of a burner management system
    /// (M44, docs/DESIGN.md §49). Clear: armed, and its fresh reading outside
    /// its condition. One that is not clear when the stop ends keeps the
    /// equipment dark for a person (`RestartBar::PermissiveNotClear`).
    ///
    /// Empty unless the file names some, and only on a trip whose reset
    /// restarts. Trips on OTHER equipment only: one on the same equipment is
    /// asked by every restart anyway (`RestartBar::TripAboutToFire`), and the
    /// loader refuses naming it.
    pub restart_permissives: Vec<TripId>,
    pub state: TripState,
    /// The measurement the last trip pass compared. `None` only before the
    /// first tick, when no pass has run — and, for a flow trip, still `None` on
    /// tick 1's snapshot, whose pass had no solution to read (§36).
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

    /// The plant's hydraulics alone: every node, pipe and exchanger coupling,
    /// with the same ids, and NO control loops or trips. For a solver's what-if
    /// solve (M48.1, docs/DESIGN.md §53: a relief held in its remembered state),
    /// which reads nothing else. Not `Clone`, for M8.2's reason: a copied loop
    /// would fork its controller's memory into a second plant.
    pub fn hydraulic_copy(&self) -> PlantGraph {
        PlantGraph {
            g: self.g.clone(),
            couplings: self.couplings.clone(),
            ..PlantGraph::default()
        }
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

    /// The total mass rate [kg/s, a magnitude] the network draws out of `node`
    /// through its SOLVED pipes, each pipe's signed flow [kg/s, positive along
    /// it] read from `flow` (M62.3, docs/DESIGN.md §67.3): what a tank's
    /// outlets share one bottom layer by (`TankState::drawn_composition`).
    /// An engine-written pipe (a boil-off vent, an overflow) is left out: its
    /// flow is written after the solve and leaves by its own rule. Summed in
    /// `incident`'s edge order, a function of the graph alone (rule 3).
    pub fn solved_outflow(&self, node: NodeId, flow: impl Fn(EdgeId) -> f64) -> f64 {
        self.incident(node)
            .into_iter()
            .filter(|&(edge, other, _)| other != node && !self.pipe(edge).leak.is_engine_written())
            .map(|(edge, _, incoming)| {
                let into_node = if incoming { flow(edge) } else { -flow(edge) };
                (-into_node).max(0.0)
            })
            .sum()
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

    /// Every column draw that runs through a LINE of pumps and coolers to its
    /// store (M62.4b, docs/DESIGN.md §67.13): the column, the draw's index on
    /// it, and the line's pipes in order from the column. Ascending by column,
    /// then by draw; empty on every plant whose draws end at their stores.
    pub fn draw_lines(&self) -> Vec<(NodeId, usize, &[EdgeId])> {
        let mut lines = Vec::new();
        for id in self.node_ids() {
            if let NodeKind::Column { draws, .. } = &self.node(id).kind {
                for (index, draw) in draws.iter().enumerate() {
                    if !draw.line.is_empty() {
                        lines.push((id, index, draw.line.as_slice()));
                    }
                }
            }
        }
        lines
    }

    /// A furnace's coil in the stream's order (M62.4c, docs/DESIGN.md §67.14):
    /// its cells from the coil's inlet, then the furnace itself, the outlet —
    /// walked back from the furnace through the cells that feed it. A furnace
    /// with no cells is its own one-member coil.
    pub fn coil_chain(&self, furnace: NodeId) -> Vec<NodeId> {
        let mut chain = vec![furnace];
        let mut at = furnace;
        while let Some(cell) = self
            .incident(at)
            .into_iter()
            .find_map(|(_, other, incoming)| match self.node(other).kind {
                NodeKind::CoilCell { furnace: f } if incoming && f == furnace => Some(other),
                _ => None,
            })
        {
            if chain.contains(&cell) {
                break;
            }
            chain.push(cell);
            at = cell;
        }
        chain.reverse();
        chain
    }

    /// Whether `edge` is a segment of a furnace's coil — between two of its
    /// cells, or its last cell and the furnace (M62.4c, docs/DESIGN.md §67.14).
    pub fn is_coil_segment(&self, edge: EdgeId) -> bool {
        let (from, to) = self.endpoints(edge);
        let coil_of = |id: NodeId| match self.node(id).kind {
            NodeKind::CoilCell { furnace } => Some(furnace),
            NodeKind::Furnace { .. } => Some(id),
            _ => None,
        };
        matches!(self.node(from).kind, NodeKind::CoilCell { .. })
            && coil_of(from).is_some()
            && coil_of(from) == coil_of(to)
    }

    /// Whether `edge` is a pipe of a draw LINE (M62.4b, docs/DESIGN.md §67.13)
    /// or of a pumparound loop's (M72.2, §77.2): prescribed, either way.
    /// Asked of every edge on every pass of a flow solve, so it answers from
    /// the edge's own ends first: a line pipe leaves a column, a pump or a
    /// cooler, and every other pipe is refused before any walk.
    pub fn is_draw_line_edge(&self, edge: EdgeId) -> bool {
        let (from, _) = self.endpoints(edge);
        if !matches!(
            self.node(from).kind,
            NodeKind::Column { .. }
                | NodeKind::Pump { .. }
                | NodeKind::Cooler { .. }
                | NodeKind::WaterCooler { .. }
        ) {
            return false;
        }
        self.node_ids().any(|id| match &self.node(id).kind {
            NodeKind::Column { draws, cascade, .. } => {
                draws.iter().any(|d| d.line.contains(&edge))
                    || cascade
                        .as_ref()
                        .is_some_and(|spec| spec.loops.iter().any(|l| l.line.contains(&edge)))
            }
            _ => false,
        })
    }

    /// Every pumparound loop's LINE (M72.2, docs/DESIGN.md §77.2): the column,
    /// the loop's index on it, and the line's pipes in order from the column,
    /// the last back into it. Ascending by column, then by loop; empty on
    /// every plant without one.
    pub fn loop_lines(&self) -> Vec<(NodeId, usize, &[EdgeId])> {
        let mut lines = Vec::new();
        for id in self.node_ids() {
            if let NodeKind::Column {
                cascade: Some(spec),
                ..
            } = &self.node(id).kind
            {
                for (index, pumparound) in spec.loops.iter().enumerate() {
                    if !pumparound.line.is_empty() {
                        lines.push((id, index, pumparound.line.as_slice()));
                    }
                }
            }
        }
        lines
    }

    /// The column and loop whose line `edge` brings back INTO its column — a
    /// loop's last pipe (M72.2, docs/DESIGN.md §77.2): the column reads it on
    /// its own, never as feed, and does not wait on it.
    pub fn loop_return(&self, edge: EdgeId) -> Option<(NodeId, usize)> {
        let (_, to) = self.endpoints(edge);
        let NodeKind::Column {
            cascade: Some(spec),
            ..
        } = &self.node(to).kind
        else {
            return None;
        };
        spec.loops
            .iter()
            .position(|l| l.line.last() == Some(&edge))
            .map(|index| (to, index))
    }

    /// The column and loop whose line `edge` leaves the column by — a loop's
    /// first pipe, carrying the draw stage's liquid (M72.2).
    pub fn loop_draw(&self, edge: EdgeId) -> Option<(NodeId, usize)> {
        let (from, _) = self.endpoints(edge);
        let NodeKind::Column {
            cascade: Some(spec),
            ..
        } = &self.node(from).kind
        else {
            return None;
        };
        spec.loops
            .iter()
            .position(|l| l.line.first() == Some(&edge))
            .map(|index| (from, index))
    }

    /// The column and loop whose cooler `node` is (M72.2): its duty is the
    /// column's to write.
    pub fn loop_cooler(&self, node: NodeId) -> Option<(NodeId, usize)> {
        if !matches!(self.node(node).kind, NodeKind::Cooler { .. }) {
            return None;
        }
        self.node_ids().find_map(|id| match &self.node(id).kind {
            NodeKind::Column {
                cascade: Some(spec),
                ..
            } => spec
                .loops
                .iter()
                .position(|l| l.cooler == Some(node))
                .map(|index| (id, index)),
            _ => None,
        })
    }

    /// The hydrogen pass running through bed `bed`, with its kinetics (M59.1,
    /// docs/DESIGN.md §64.2), or `None` for a bed without one — and for any node
    /// that is not a bed. The loader admits at most one pass per bed; the lowest
    /// id wins on a graph built by hand with two.
    pub fn hydrogen_pass_of(&self, bed: NodeId) -> Option<(NodeId, HydrogenKinetics)> {
        self.node_ids().find_map(|id| match &self.node(id).kind {
            NodeKind::BedHydrogen {
                bed: b, kinetics, ..
            } if *b == bed => Some((id, *kinetics)),
            _ => None,
        })
    }

    /// The node the heat sweep resolves with `id` as one vertex: an exchanger
    /// side's partner (§4a), a hydrogen pass's bed, or a bed's hydrogen pass
    /// (M59.1, docs/DESIGN.md §64.2). `None` for every other node.
    pub fn sweep_partner(&self, id: NodeId) -> Option<NodeId> {
        if let Some((partner, _)) = self.exchanger_partner(id) {
            return Some(partner);
        }
        match &self.node(id).kind {
            NodeKind::BedHydrogen { bed, .. } => Some(*bed),
            NodeKind::BedReactor { .. } => self.hydrogen_pass_of(id).map(|(pass, _)| pass),
            _ => None,
        }
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
            // A coil is watched by trips only, which range-check their own limits
            // (`check_trip_limit`); a loop on one is refused at load. Rule 5's
            // backstop, not a message anyone should meet.
            (MeasurementPoint::Coil(node), _) => {
                return Err(SimError::InvalidCommand(format!(
                    "the coil of '{}' is watched by trips only, so it has no setpoint range",
                    self.node(node).name
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
            // A catalyst bed's temperature is a state (M57.4, docs/DESIGN.md
            // §62.11), with absolute zero its only bound as well.
            (
                ControlledValue::Temperature { k },
                NodeKind::Tank(_)
                | NodeKind::Vessel(_)
                | NodeKind::Furnace { .. }
                | NodeKind::Cooler { .. }
                | NodeKind::WaterCooler { .. }
                | NodeKind::BedReactor { .. },
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

    /// A quench check's two temperatures, as they stand now (M57.4,
    /// docs/DESIGN.md §62.11). **The single owner of the comparison**: the
    /// loader reads it at load, the tick's loop pass at the top of every tick,
    /// and `Command::SetControllerMode`'s refusal fresh. `hydraulics` is the
    /// last hydraulic solution, `None` before the first tick.
    ///
    /// A pipe's stream temperature is a reading only while that pipe carries a
    /// forward flow: a still pipe keeps the last temperature it carried. So a
    /// quench that is not flowing reads its origin's temperature, and a main
    /// stream that is not flowing reads the bed's own — the quench is then the
    /// bed's only feed, and cools it exactly when it is colder than it.
    ///
    /// # Errors
    /// `SimError::Numerical` if either temperature is not finite, or if the
    /// check names an origin or a bed of the wrong kind (unreachable: the loader
    /// builds every check).
    pub fn quench_reading(
        &self,
        check: &QuenchCheck,
        hydraulics: Option<&crate::traits::HydraulicSolution>,
    ) -> Result<QuenchReading, SimError> {
        let flowing = |pipe: EdgeId| -> bool {
            hydraulics
                .and_then(|solution| solution.edge_mass_flow.get(&pipe))
                .is_some_and(|flow| *flow > 0.0)
        };
        let quench_k = if flowing(check.quench_pipe) {
            self.pipe(check.quench_pipe).stream.temperature
        } else {
            match &self.node(check.origin).kind {
                NodeKind::Tank(tank) => tank.temperature,
                NodeKind::Source { temperature, .. } => *temperature,
                // A hydrogen quench's separator (M59.3, docs/DESIGN.md §64.12).
                NodeKind::Vessel(vessel) => vessel.temperature,
                _ => {
                    return Err(SimError::Numerical(format!(
                        "internal: a quench check starts at '{}', which is not a tank, a source \
                         or a vessel; the loader refuses that (docs/DESIGN.md §62.11)",
                        self.node(check.origin).name
                    )))
                }
            }
        };
        let joins_k = if flowing(check.main_pipe) {
            self.pipe(check.main_pipe).stream.temperature
        } else {
            match &self.node(check.bed).kind {
                NodeKind::BedReactor { bed, .. } => bed.temperature,
                _ => {
                    return Err(SimError::Numerical(format!(
                        "internal: a quench check measures '{}', which is not a catalyst bed; \
                         the loader refuses that (docs/DESIGN.md §62.11)",
                        self.node(check.bed).name
                    )))
                }
            }
        };
        if !quench_k.is_finite() || !joins_k.is_finite() {
            return Err(SimError::Numerical(format!(
                "a quench check on '{}' read {} K against {} K, which cannot be compared",
                self.node(check.bed).name,
                quench_k.value(),
                joins_k.value()
            )));
        }
        Ok(QuenchReading { quench_k, joins_k })
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
    /// applies is "no measurement, no action" (§23 fork 2). A COOLER's outlet is
    /// also absent while it is STAGNANT: a zero-volume node with no inflow holds
    /// a placeholder, which `NodeStates::held` records and this reader refuses to
    /// pass off as a measurement (§23 fork 4). A FURNACE's is not, since M34: with
    /// no flow it reads its coil, a state the tick computed (docs/DESIGN.md §37).
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
            // **A furnace's coil: a STATE, present from load** (M35, docs/DESIGN.md
            // §39). Read off the graph, where `Engine::tick` writes it at the end of
            // every tick, so it is never absent — not before the first tick, not
            // with no flow.
            (MeasurementPoint::Coil(node), MeasuredVariable::Temperature) => {
                return match &self.node(node).kind {
                    NodeKind::Furnace { coil, .. } => Ok(Some(ControlledValue::Temperature {
                        k: coil.temperature,
                    })),
                    _ => Err(SimError::Scenario(format!(
                        "'{}' is not a furnace, so it has no coil: a coil is a furnace's tube \
                         metal (docs/DESIGN.md §37)",
                        self.node(node).name
                    ))),
                };
            }
            (MeasurementPoint::Coil(node), _) => {
                return Err(SimError::Scenario(format!(
                    "the coil of '{}' is metal with a temperature, and has no {}: a coil \
                     is measured for its temperature only (docs/DESIGN.md §39)",
                    self.node(node).name,
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
            // before the first one, and — for a cooler — absent while the unit is
            // stagnant, when its entry is a held placeholder rather than a
            // computed value. Both absences are `None`, and the caller holds the
            // loop on either. A furnace is never held since its coil (M34, §37).
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
                // A check valve is a valve that is OPEN in normal operation, so
                // the relief valve's reason below does not apply to it: it is
                // refused with the other pass-throughs, on scope alone.
                | NodeKind::CheckValve { .. }
                | NodeKind::HeatExchanger
                // A water cooler (M62.4b) on the same scope: no loop needs its
                // outlet yet, and its actuator would be the water's flow.
                | NodeKind::WaterCooler { .. }
                | NodeKind::CoilCell { .. }
                // A compressor and a bed's hydrogen pass (M59.1) are zero-volume
                // gas pass-throughs: refused on the same scope. The bed itself
                // is what a loop measures.
                | NodeKind::Compressor { .. }
                | NodeKind::BedHydrogen { .. },
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
            // **A catalyst bed: a STATE, present from load** (M57.3, docs/DESIGN.md
            // §62.10), read off the graph as a furnace's coil is, where
            // `Engine::tick` writes it at the end of every tick — so never absent,
            // flowing or not. Read for a TRIP: a loop on a bed is refused by the
            // loader, by name, until M57.4 argues one.
            (MeasuredVariable::Temperature, NodeKind::BedReactor { bed, .. }) => {
                Ok(Some(ControlledValue::Temperature { k: bed.temperature }))
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
    /// name or the pipe's — and a coil's, its furnace's.
    pub fn point_name(&self, point: MeasurementPoint) -> &str {
        match point {
            MeasurementPoint::Node(node) | MeasurementPoint::Coil(node) => &self.node(node).name,
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
            (NodeKind::Cooler { duty } | NodeKind::Furnace { duty, .. }, Some(max)) => {
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
            (NodeKind::Cooler { duty } | NodeKind::Furnace { duty, .. }, Some(max)) => {
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

    /// M62.3's tank layers (docs/DESIGN.md §67.3): an oil of 800 kg/m³ over
    /// free water, in a 1 m² tank 10 m tall.
    mod water_layer {
        use super::*;
        use crate::components::PseudoComponent;

        fn slate(free: bool) -> Slate {
            let oil = PseudoComponent {
                name: "oil".into(),
                density: Some(KgPerM3(800.0)),
                ..PseudoComponent::water()
            };
            let water = PseudoComponent {
                free_water: free,
                ..PseudoComponent::water()
            };
            Slate::new(vec![oil, water]).unwrap()
        }

        fn tank(mass: f64, water: f64) -> TankState {
            TankState {
                area: SquareMeter(1.0),
                height: Meter(10.0),
                mass: Kg(mass),
                temperature: Kelvin(300.0),
                composition: Composition::from_weights(&[1.0 - water, water]).unwrap(),
                ambient_ua: WattPerKelvin::ZERO,
            }
        }

        /// The outlet takes the water first: pure water while the draw is
        /// within the layer, all of it and the oil after on the tick it runs
        /// out, the whole inventory past empty — and the inventory itself
        /// where no water stands or the slate has no free water.
        #[test]
        fn the_outlet_draws_the_water_first() {
            let wet = slate(true);
            let t = tank(1000.0, 0.1); // 100 kg of water under 900 kg of oil
            let pure = t.drawn_composition(&wet, Kg(40.0)).unwrap().unwrap();
            assert_eq!(pure.fractions(), &[0.0, 1.0]);
            let edge = t.drawn_composition(&wet, Kg(100.0)).unwrap().unwrap();
            assert_eq!(edge.fractions(), &[0.0, 1.0]);
            let past = t.drawn_composition(&wet, Kg(250.0)).unwrap().unwrap();
            assert!(
                (past.fractions()[1] - 0.4).abs() < 1e-15,
                "{:?}",
                past.fractions()
            );
            let all = t.drawn_composition(&wet, Kg(5000.0)).unwrap().unwrap();
            assert!((all.fractions()[1] - 0.1).abs() < 1e-15);
            assert!(tank(1000.0, 0.0)
                .drawn_composition(&wet, Kg(40.0))
                .unwrap()
                .is_none());
            assert!(t
                .drawn_composition(&slate(false), Kg(40.0))
                .unwrap()
                .is_none());
        }

        /// The layer's height `m_w/(ρ_w·A)`, zero on a wet slate with no water
        /// in the tank, and not reported on a slate without free water.
        #[test]
        fn the_layer_has_a_height() {
            let h = tank(1000.0, 0.1).water_level(&slate(true)).unwrap();
            assert!((h.value() - 100.0 / 998.0).abs() < 1e-15);
            assert_eq!(
                tank(1000.0, 0.0).water_level(&slate(true)),
                Some(Meter(0.0))
            );
            assert_eq!(tank(1000.0, 0.1).water_level(&slate(false)), None);
        }

        /// Over the brim goes the top layer: oil alone while oil is above the
        /// excess, leaving the liquid exactly at the brim; all the oil and the
        /// water above the brim after it once the water reaches it; water
        /// alone from a tank of water. Nothing at or below the brim, nothing
        /// from a dry tank (the dry rule spills it).
        #[test]
        fn the_brim_spills_the_top_layer() {
            let wet = slate(true);
            // 998 kg of water (1 m) under 7 300 kg of oil (9.125 m): 0.125 m over.
            let over = tank(998.0 + 7300.0, 998.0 / 8298.0);
            let (spill, what) = over.top_spill(&wet).unwrap().unwrap();
            assert_eq!(what.fractions(), &[1.0, 0.0]);
            assert!((spill.value() - 100.0).abs() < 1e-9, "{}", spill.value());
            // 9 980 kg of water (10 m) under 160 kg of oil (0.2 m): all the
            // oil, exactly, and none of the water, which stands at the brim.
            let brimful = tank(9980.0 + 160.0, 9980.0 / 10140.0);
            let (spill, what) = brimful.top_spill(&wet).unwrap().unwrap();
            assert!((spill.value() - 160.0).abs() < 1e-9, "{}", spill.value());
            assert!(what.fractions()[1].abs() < 1e-12);
            // 10 179.6 kg of water (10.2 m) under 80 kg of oil (0.1 m): every
            // drop of the oil, and the 0.2 m of water above the brim after it.
            let flooded = tank(10_179.6 + 80.0, 10_179.6 / 10_259.6);
            let (spill, what) = flooded.top_spill(&wet).unwrap().unwrap();
            assert!((spill.value() - 279.6).abs() < 1e-9, "{}", spill.value());
            assert!((what.fractions()[0] * spill.value() - 80.0).abs() < 1e-9);
            // 10 179.6 kg of water: 0.2 m of it over the brim.
            let water = tank(10_179.6, 1.0);
            let (spill, what) = water.top_spill(&wet).unwrap().unwrap();
            assert_eq!(what.fractions(), &[0.0, 1.0]);
            assert!((spill.value() - 199.6).abs() < 1e-9);
            assert!(tank(998.0 + 7000.0, 998.0 / 7998.0)
                .top_spill(&wet)
                .unwrap()
                .is_none());
            assert!(tank(8100.0, 0.0).top_spill(&wet).unwrap().is_none());
        }

        /// A tank that has spilled stands AT its brim, not a rounding over it:
        /// spilled once, as `Engine::tick` debits it, it spills nothing on the
        /// next tick — the tie M23's dry overflow keeps by construction, so a
        /// full wet tank whose inflow stopped does not spill crumbs for ever.
        /// Swept over fill heights and water shares, as the rounding is the
        /// question.
        #[test]
        fn a_spilled_tank_stands_at_its_brim() {
            let wet = slate(true);
            for i in 0..400 {
                let water_share = 0.01 + 0.002 * f64::from(i % 50);
                let over = 1.0 + 1e-4 * f64::from(i) + 1e-7;
                let mut t = tank(1.0, water_share);
                t.mass = Kg(t.capacity(&wet).value() * over);
                let held = t.mass.value();
                let (spill, _) = t.spill_top(&wet).unwrap().unwrap();
                assert!((t.mass.value() + spill.value() - held).abs() <= 1e-12 * held);
                assert!(
                    t.top_spill(&wet).unwrap().is_none(),
                    "case {i}: {} kg left over a {} kg brim",
                    t.mass.value(),
                    t.capacity(&wet).value()
                );
            }
        }
    }

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
