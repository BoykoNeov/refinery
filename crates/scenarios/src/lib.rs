//! refinery-scenarios: plant-definition file format (TOML) and the
//! EngineBuilder that instantiates an Engine from it — including the
//! fidelity selection (which solver implementations to plug in).
//!
//! The scenario file is the ONLY place fidelity is chosen. See
//! scenarios/tank_pump_valve.toml for the reference example of the format.
//!
//! Unit conventions in scenario files (converted to SI on load):
//! - pressures in bar (absolute), temperatures in °C, lengths in m,
//!   volumes in m³, flow coefficients as customary metric Kv (m³/h at 1 bar)
//!   — human-friendly at the boundary, SI inside, per CLAUDE.md rule 4.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::error::SimError;
use refinery_core::graph::{
    ColumnDraw, HeatExchangerCoupling, LeakRole, Node, NodeId, NodeKind, Pipe, PlantGraph,
    TankState, VesselState,
};
use refinery_core::stream::Stream;
use refinery_core::traits::{FlowSolver, ReactionModel, SeparationModel, ThermoModel};
use refinery_core::units::{
    CubicMeter, JPerKgK, Kelvin, Kg, KgPerM3, KgPerMol, Meter, Pascal, Seconds, SquareMeter, Watt,
    WattPerKelvin, P_ATM, T_AMBIENT,
};
use serde::Deserialize;
use std::collections::BTreeMap;

#[derive(Debug, Deserialize)]
pub struct ScenarioFile {
    pub meta: Meta,
    pub simulation: Simulation,
    pub fidelity: Fidelity,
    /// Node definitions keyed by unique name; IndexMap preserves file order
    /// so node ids are deterministic and human-predictable.
    pub nodes: indexmap::IndexMap<String, NodeDef>,
    pub pipes: Vec<PipeDef>,
    /// Thermal pairings between `heat_exchanger` nodes. Optional: a plant with
    /// no exchangers needs no table.
    #[serde(default)]
    pub exchangers: Vec<ExchangerDef>,
    /// The pseudo-component slate, in canonical order. Optional: absent means
    /// the water-only slate every scenario written before M3 meant, which is
    /// what keeps those files bit-identical (see `build_engine` step 1).
    #[serde(default)]
    pub components: Vec<ComponentDef>,
}

/// One `[[components]]` entry: a boiling-point cut.
///
/// File ORDER is the canonical slate order, exactly as `[nodes]` order fixes
/// node ids — `Composition` is a positional vector of mass fractions, so the
/// slate's order is part of the engine's identity and must not depend on a hash
/// or on sorting by name. Compositions elsewhere in the file are written by
/// NAME and resolved against this order, so a reader never has to count
/// positions.
#[derive(Debug, Deserialize)]
pub struct ComponentDef {
    pub name: String,
    /// True boiling point of the cut [°C at this boundary, K inside].
    pub tb_c: f64,
    pub molar_mass_kg_per_mol: f64,
    /// Liquid density at reference conditions [kg/m³].
    ///
    /// Required for a liquid-phase component and REFUSED for a gas-phase one,
    /// whose density is `P·M̄/(R·T)`: a declared constant there would be an
    /// authoritative-looking number no code reads, which is the same failure
    /// mode as an invented kinetic constant (docs/DESIGN.md §3a).
    #[serde(default)]
    pub density_kg_per_m3: Option<f64>,
    pub cp_j_per_kg_k: f64,
    /// `"liquid"` (default) or `"gas"`. Absent means liquid, which is what
    /// every slate written before M5.2 meant — the default that keeps those
    /// files bit-identical rather than merely still-loading.
    #[serde(default)]
    pub phase: Option<String>,
}

/// One `[[exchangers]]` entry: which two sides are thermally coupled, and how
/// effectively.
///
/// A table of its own rather than a field on the node, because effectiveness is
/// a property of the PAIR. Written on the nodes it would have to be repeated,
/// and two halves of one exchanger could then disagree — an inconsistency the
/// loader would have to detect. Here it cannot be expressed.
#[derive(Debug, Deserialize)]
pub struct ExchangerDef {
    pub side_a: String,
    pub side_b: String,
    /// ε ∈ (0, 1]. See `HeatExchangerCoupling::effectiveness`.
    pub effectiveness: f64,
}

#[derive(Debug, Deserialize)]
pub struct Meta {
    pub name: String,
    #[serde(default)]
    pub description: String,
}

#[derive(Debug, Deserialize)]
pub struct Simulation {
    /// Tick length [s].
    pub dt: f64,
}

#[derive(Debug, Deserialize)]
pub struct Fidelity {
    /// "newton" | "simple"
    pub flow: String,
    /// "constant" — the only selectable value, and `"trouton"` joins it in M7.3.
    ///
    /// Until M7.2 this string was parsed and then **ignored**: `build_engine`
    /// hardcoded `ConstantThermo`, so `thermo = "nonsense"` loaded a working
    /// plant. It was alone among the fidelity keys in that, and it was found by
    /// wiring `separation` beside it in M7.1 rather than by a test — nothing
    /// reached the value, so nothing could fail on it.
    ///
    /// **Why `TroutonThermo` exists in `solvers` but cannot be named here yet.**
    /// The only consumer of a K-value is the cascade, which is M7.3. Until then
    /// the cut-point splitter ignores the thermo model entirely, so selecting
    /// `"trouton"` would change no number in any plant — a scenario knob nothing
    /// can discriminate, which is precisely the shape M7.1 measured on
    /// `smearing_k` (`a-hand-written-scenario-can-be-vacuous`). The arm lands in
    /// M7.3 together with the load-time refusal of the pairing it makes
    /// possible: `separation = "cascade"` with `thermo = "constant"`, which
    /// would otherwise fail on the first tick instead of at load.
    #[serde(default = "default_constant")]
    pub thermo: String,
    /// "none" (M1) — expands in M4
    #[serde(default = "default_none")]
    pub reactions: String,
    /// "cut_point" (M3.2) | "cascade" (M7.3).
    ///
    /// Defaults to the boiling-range splitter, which is what every file written
    /// before M7 MEANS — not merely what keeps them loading. That distinction is
    /// the M5.2 `phase` precedent: a default chosen so old files stay
    /// bit-identical, rather than one chosen so they still parse. A file with no
    /// column is unaffected either way, since the model is called once per column
    /// and never otherwise.
    #[serde(default = "default_cut_point")]
    pub separation: String,
}
fn default_constant() -> String {
    "constant".into()
}
fn default_none() -> String {
    "none".into()
}
fn default_cut_point() -> String {
    "cut_point".into()
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum NodeDef {
    Source {
        pressure_bar: f64,
        temperature_c: f64,
        /// Mass-fraction weights keyed by component name; normalized on load.
        ///
        /// Optional, and the default is deliberately NOT "component 0" — it is
        /// "the only component, if there is only one". A one-component slate has
        /// exactly one meaning for an absent composition, so omitting it is
        /// unambiguous; with a real crude slate, defaulting to the first cut
        /// would silently make a crude source pure light naphtha, which is a
        /// plant that runs and is wrong. See `resolve_composition`.
        #[serde(default)]
        composition: Option<BTreeMap<String, f64>>,
    },
    Sink {
        pressure_bar: f64,
        /// Temperature of the fluid the sink returns under reverse flow.
        /// Optional: a sink that never back-feeds is unaffected by it, so
        /// requiring it in every file would be noise. Defaults to ambient.
        #[serde(default = "default_ambient_c")]
        temperature_c: f64,
        /// Composition of the fluid the sink returns under reverse flow.
        /// Optional on a one-component slate only — see the `Source` field and
        /// `node_kind`'s Sink arm for why this one cannot default the way its
        /// temperature does.
        #[serde(default)]
        composition: Option<BTreeMap<String, f64>>,
    },
    Atmosphere,
    Tank {
        area_m2: f64,
        height_m: f64,
        initial_level_m: f64,
        temperature_c: f64,
        /// Ambient heat transfer coefficient × area, `UA` [W/K]. Already SI —
        /// there is no customary unit for it worth converting from, unlike the
        /// bar/°C/MW elsewhere in this file.
        ///
        /// Optional, defaulting to 0: a perfectly insulated tank, which is what
        /// every scenario written before this field existed meant. See
        /// `TankState::ambient_ua`.
        #[serde(default)]
        ambient_ua_w_per_k: f64,
        /// Initial tank contents. Same rule as `Source::composition`.
        #[serde(default)]
        composition: Option<BTreeMap<String, f64>>,
    },
    /// Capacitive gas vessel (docs/DESIGN.md §3a fork 2): knock-out drum,
    /// receiver, blowdown vessel. Gas only — a liquid holdup is a `tank`.
    ///
    /// It declares a PRESSURE, not an inventory, because that is the number an
    /// operator knows about a vessel and the number its state actually is; the
    /// mass follows from `m = P·V·M̄/(R·T)`. The tank is the mirror image and for
    /// the same reason: it declares a LEVEL, and its mass follows from `ρ·A·h`.
    Vessel {
        volume_m3: f64,
        /// Initial pressure [bar]. The vessel's state; its mass is derived.
        pressure_bar: f64,
        temperature_c: f64,
        /// Initial contents. Same rule as `Source::composition`, and it must be
        /// gas-phase — see `node_kind`'s Vessel arm.
        #[serde(default)]
        composition: Option<BTreeMap<String, f64>>,
    },
    Pump {
        h0_m: f64,
        a: f64,
        #[serde(default = "default_true")]
        on: bool,
    },
    Valve {
        kv: f64,
        opening: f64,
        /// IEC 60534-2-1's pressure differential ratio factor. REQUIRED in gas
        /// service and REFUSED in liquid service, both decided by the
        /// topological single-phase analysis M5.2 already builds — see
        /// `require_gas_valve_x_t`.
        x_t: Option<f64>,
    },
    /// Spring-loaded pressure safety valve. Set pressure and accumulation band in
    /// BAR ABSOLUTE and BAR respectively, the units a relief datasheet uses,
    /// converted at this boundary like every other human-friendly quantity.
    ReliefValve {
        kv: f64,
        set_pressure_bar: f64,
        /// Band above the set pressure over which the valve reaches full lift.
        /// Must be > 0: a zero band is a step, and a step is a discontinuous
        /// characteristic this solver's Jacobian is not entitled to.
        accumulation_bar: f64,
        x_t: Option<f64>,
    },
    /// Fired heater. Duty in MW — the unit refinery heaters are actually
    /// specified in, converted to W at this boundary like every other
    /// human-friendly quantity in the file.
    Furnace {
        duty_mw: f64,
    },
    /// Cooler. Duty in MW REMOVED from the stream — a positive magnitude, like
    /// a furnace's. Use this rather than a negative `furnace` duty; the loader
    /// rejects those.
    Cooler {
        duty_mw: f64,
    },
    /// One side of a two-stream heat exchanger. Carries no parameters: the
    /// pairing and its effectiveness live in an `[[exchangers]]` entry, which
    /// every side must appear in exactly once.
    HeatExchanger,
    Junction,
    /// Fixed cut-point distillation column (simple fidelity). One feed in, N
    /// draws out, split by boiling range (docs/DESIGN.md §5).
    Column {
        /// Operating pressure [bar]. Pinned like a source/sink/tank; real columns
        /// run on pressure control and the overhead pressure sets the cut
        /// structure.
        pressure_bar: f64,
        /// Ramp width across each cut point [K]. A temperature WIDTH, so no
        /// °C→K offset (a delta of 10 °C is 10 K). `0` is a sharp splitter.
        #[serde(default)]
        smearing_k: f64,
        /// Draws in ascending boiling-point order, lightest first. Each names the
        /// product node it feeds and the top of its boiling band. See `DrawDef`.
        draws: Vec<DrawDef>,
    },
    /// Isothermal conversion reactor (simple fidelity). One feed in, one product
    /// out, held at `t_set_c`; the chemistry comes from the engine's selected
    /// `ReactionModel` (`[fidelity].reactions`). Hydraulically a furnace
    /// (docs/DESIGN.md §5).
    Reactor {
        /// Held reactor-outlet temperature [°C] — the ROT setpoint imposed on the
        /// product stream.
        t_set_c: f64,
        /// Residence time [s]. Passed to the reaction model; the lookup fidelity
        /// ignores it, the M4.2 kinetics integrate over it.
        tau_s: f64,
    },
}

/// One `draws = [...]` entry of a `column` node.
///
/// The list order is the cut order: draw `i`'s band runs from the previous
/// draw's `up_to_c` (or −∞ for the lightest) up to its own. The heaviest draw
/// OMITS `up_to_c` — it is the open-topped catch-all that takes everything above
/// the last cut, so no component can fall off the end and be lost. Every other
/// draw must set it, strictly increasing. Outlets are addressed by name, matching
/// the slate's positional-vector / name-addressed-file convention.
#[derive(Debug, Deserialize)]
pub struct DrawDef {
    /// Name of the product node this draw feeds.
    pub outlet: String,
    /// Top of this draw's boiling band [°C, absolute]. Omitted on (and only on)
    /// the heaviest, last draw.
    #[serde(default)]
    pub up_to_c: Option<f64>,
}
fn default_true() -> bool {
    true
}
/// Ambient in the scenario file's display units (°C), so the default round-trips
/// through `c_to_k` to exactly `T_AMBIENT` rather than to a near-miss constant.
fn default_ambient_c() -> f64 {
    T_AMBIENT.value() - 273.15
}

#[derive(Debug, Deserialize)]
pub struct PipeDef {
    pub name: String,
    pub from: String,
    pub to: String,
    pub length_m: f64,
    pub diameter_m: f64,
    #[serde(default = "default_friction")]
    pub friction_factor: f64,
    #[serde(default)]
    pub elevation_change_m: f64,
    /// Ambient heat transfer coefficient × exposed area, `UA` [W/K].
    ///
    /// Optional and defaulting to 0 — a perfectly insulated pipe — for the same
    /// reason as the tank's, and it matters more here: a pipe is the one body
    /// EVERY scenario has, so a nonzero default would change the answer of every
    /// file ever written rather than only those with tanks. See
    /// `Pipe::ambient_ua` for why this drives a transform and not a heat term.
    #[serde(default)]
    pub ambient_ua_w_per_k: f64,
    /// Declares this pipe punctureable, naming the `Atmosphere` node its leak
    /// vents to. Absent (the default) = a pipe that cannot be damaged.
    ///
    /// **Declared rather than automatic, and the discriminator is not CPU.** A
    /// leak path is built at LOAD (docs/DESIGN.md §3b): the pipe is split in
    /// half, a `Junction` joins the halves, and a dormant orifice hangs off that
    /// junction. Auto-creating one for every pipe would therefore double every
    /// pipe and add a junction per pipe in every scenario ever written — every
    /// reference plant would become a different plant, which is large in meaning
    /// even though it is small in test count. A game that wants
    /// puncture-anywhere declares a path on every pipe in its own file instead of
    /// imposing one on the reference plants.
    ///
    /// The node is named rather than found, so a plant with two atmospheres (an
    /// enclosure and the outside, say) stays expressible and no scenario acquires
    /// an implicit one it did not write.
    #[serde(default)]
    pub leak_to: Option<String>,
}
fn default_friction() -> f64 {
    0.02
}

pub fn load_str(toml_src: &str) -> Result<ScenarioFile, SimError> {
    toml::from_str(toml_src).map_err(|e| SimError::Scenario(e.to_string()))
}

/// Build a runnable engine from a scenario. Steps:
/// 1. Build the Slate (water-only until M3's [components] table exists).
/// 2. Instantiate nodes in file order (unit conversion at this boundary),
///    then pipes, resolving names → NodeIds; unknown names are errors.
/// 3. Validate topology: pumps/valves have exactly 1 in + 1 out edge;
///    every node reachable; at least one pressure-fixing node per
///    connected component (otherwise the hydraulic problem is singular —
///    fail at load with a clear message, not at solve with divergence).
/// 4. Select solver impls from [fidelity]; unknown names are errors
///    listing valid options.
pub fn build_engine(scenario: &ScenarioFile) -> Result<Engine, SimError> {
    // Step 1: slate. An absent [[components]] table means the water-only slate,
    // which is what every scenario written before M3 meant — that default is
    // what keeps those files bit-identical rather than merely still-loading.
    let slate = build_slate(&scenario.components)?;

    // Step 2: instantiate nodes in file order (IndexMap preserves it, so node
    // ids are deterministic), then pipes — resolving names → NodeIds. Unit
    // conversion happens here, at the human-friendly ↔ SI boundary.
    let mut graph = PlantGraph::new();
    for (name, def) in &scenario.nodes {
        validate_node_def(name, def)?;
        let kind = node_kind(name, def, &slate)?;
        graph.add_node(Node {
            name: name.clone(),
            kind,
            heat_input: Watt::ZERO,
        });
    }
    // Step 2a: resolve column draws now that every node exists — a draw may name
    // an outlet defined later in the file, exactly like an exchanger coupling.
    // This is where a draw's outlet must be a pressure-fixing node (the
    // free-node-on-a-draw-line rejection, DESIGN §5); the pipe-level topology
    // (one pipe per draw, correct direction) is checked in `validate_topology`
    // once the pipes exist.
    resolve_column_draws(&mut graph, scenario)?;

    for pipe in &scenario.pipes {
        validate_pipe_def(pipe)?;
        let from = graph.find_node(&pipe.from).ok_or_else(|| {
            SimError::Scenario(format!(
                "pipe '{}' references unknown 'from' node '{}'",
                pipe.name, pipe.from
            ))
        })?;
        let to = graph.find_node(&pipe.to).ok_or_else(|| {
            SimError::Scenario(format!(
                "pipe '{}' references unknown 'to' node '{}'",
                pipe.name, pipe.to
            ))
        })?;
        let whole = Pipe {
            name: pipe.name.clone(),
            length: Meter(pipe.length_m),
            diameter: Meter(pipe.diameter_m),
            friction_factor: pipe.friction_factor,
            elevation_change: Meter(pipe.elevation_change_m),
            leak: LeakRole::None,
            ambient_ua: WattPerKelvin(pipe.ambient_ua_w_per_k),
            // The solver overwrites mass_flow each tick, and transport
            // overwrites the temperature. Seed representative T/P at
            // ambient / atmospheric.
            stream: Stream::stagnant(slate.len(), T_AMBIENT, P_ATM),
        };
        match &pipe.leak_to {
            None => {
                graph.add_pipe(from, to, whole);
            }
            Some(atmosphere) => split_for_leak(&mut graph, pipe, from, to, whole, atmosphere)?,
        }
    }

    // Step 2b: thermally pair the exchanger sides, after every node exists so
    // both ends of a coupling can be resolved regardless of file order.
    build_couplings(&mut graph, &scenario.exchangers)?;

    // Step 3: validate topology at load — a clear error here beats solve-time
    // divergence for the same structural fault.
    validate_topology(&graph, &slate)?;

    // Step 3a: the single-phase connected-component guard, which also tells each
    // pipe which phase it carries. Seeding the stream's composition needs the
    // topology, so it happens here rather than in the pipe loop above; the value
    // is a tick-1 density seed only (see `seed_component_index`).
    let phases = plant_phases(&graph, &slate)?;
    require_gas_valve_x_t(&graph, &phases)?;
    refuse_gas_leak(&graph, &phases)?;
    for eid in graph.edge_ids().collect::<Vec<_>>() {
        let (src, _) = graph.endpoints(eid);
        let index = seed_component_index(&slate, phases[src.0 as usize]);
        graph.pipe_mut(eid).stream.composition = Composition::pure(slate.len(), index);
    }

    // Step 4: select solver impls from [fidelity]; unknown names are errors
    // listing the valid options.
    let flow: Box<dyn FlowSolver> = match scenario.fidelity.flow.as_str() {
        "newton" => Box::new(refinery_solvers::NewtonFlowSolver::default()),
        "simple" => Box::new(refinery_solvers::SimpleFlowSolver::default()),
        other => {
            return Err(SimError::Scenario(format!(
                "unknown flow solver '{other}' (valid: newton, simple)"
            )))
        }
    };
    let thermo: Box<dyn ThermoModel> = match scenario.fidelity.thermo.as_str() {
        "constant" => Box::new(refinery_solvers::ConstantThermo),
        other => {
            return Err(SimError::Scenario(format!(
                "unknown thermo model '{other}' (valid: constant)"
            )))
        }
    };
    let reactions: Box<dyn ReactionModel> = match scenario.fidelity.reactions.as_str() {
        "none" => Box::new(refinery_solvers::NoReactions),
        // The FCC placeholder table (M4.1). Both reacting fidelities resolve
        // their lumps against the slate by name, so an absent lump is a
        // load-time error, not a solve-time surprise.
        "lookup" => Box::new(refinery_solvers::SimpleLookup::fcc_demo(&slate)?),
        // FCC 4-lump Arrhenius kinetics (M4.2), integrated with fixed-count RK4
        // over the reactor's `tau_s`.
        "fcc" => Box::new(refinery_solvers::FourLump::fcc(&slate)?),
        other => {
            return Err(SimError::Scenario(format!(
                "unknown reaction model '{other}' (valid: none, lookup, fcc)"
            )))
        }
    };

    let separation: Box<dyn SeparationModel> = match scenario.fidelity.separation.as_str() {
        // M3.2's boiling-range splitter, and the default (see `Fidelity`).
        "cut_point" => Box::new(refinery_solvers::CutPointSplitter),
        other => {
            return Err(SimError::Scenario(format!(
                "unknown separation model '{other}' (valid: cut_point)"
            )))
        }
    };

    let config = EngineConfig {
        dt: refinery_core::units::Seconds(scenario.simulation.dt),
    };
    Ok(Engine::new(
        graph, slate, config, flow, thermo, reactions, separation,
    ))
}

/// Build the canonical slate from the `[[components]]` table, in file order.
///
/// An empty table is the water-only slate rather than an error: that is what
/// every pre-M3 scenario means, and making it explicit would churn every file
/// for no gain. Duplicate names ARE an error — `Composition` is written by name,
/// so two cuts called the same thing make a composition ambiguous, and
/// `Slate::index_of` would silently resolve every mention to the first.
fn build_slate(defs: &[ComponentDef]) -> Result<Slate, SimError> {
    if defs.is_empty() {
        return Ok(Slate::water_only());
    }
    for (i, def) in defs.iter().enumerate() {
        if defs[..i].iter().any(|d| d.name == def.name) {
            return Err(SimError::Scenario(format!(
                "component '{}' is defined twice: names must be unique, because \
                 compositions reference components by name",
                def.name
            )));
        }
        // Every property is a positive physical magnitude; a zero density would
        // divide by zero in `mixture_density`, and a zero cp makes any heat
        // input an infinite temperature rise.
        for (field, value) in [
            ("tb_c", def.tb_c + 273.15),
            ("molar_mass_kg_per_mol", def.molar_mass_kg_per_mol),
            ("cp_j_per_kg_k", def.cp_j_per_kg_k),
        ]
        .into_iter()
        .chain(def.density_kg_per_m3.map(|d| ("density_kg_per_m3", d)))
        {
            if !value.is_finite() || value <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "component '{}' has a non-positive or non-finite {field} \
                     ({value}); every component property must be > 0",
                    def.name
                )));
            }
        }
        // Phase ↔ density correspondence, refused in BOTH directions so neither
        // mistake can produce a plant that loads: a liquid with no density has
        // no density law at all, and a gas with one carries a number nothing
        // reads (docs/DESIGN.md §3a).
        match (component_phase(def)?, def.density_kg_per_m3) {
            (Phase::Liquid, None) => {
                return Err(SimError::Scenario(format!(
                    "liquid component '{}' has no density_kg_per_m3; a liquid's \
                     density is a declared constant at this fidelity",
                    def.name
                )))
            }
            (Phase::Gas, Some(rho)) => {
                return Err(SimError::Scenario(format!(
                    "gas component '{}' declares density_kg_per_m3 = {rho}, which \
                     nothing reads: a gas density is P·M̄/(R·T), computed from \
                     molar_mass_kg_per_mol and the solved pressure. Remove the field.",
                    def.name
                )))
            }
            _ => {}
        }
    }
    Slate::new(
        defs.iter()
            .map(|d| {
                Ok(PseudoComponent {
                    name: d.name.clone(),
                    tb: c_to_k(d.tb_c),
                    molar_mass: KgPerMol(d.molar_mass_kg_per_mol),
                    density: d.density_kg_per_m3.map(KgPerM3),
                    cp: JPerKgK(d.cp_j_per_kg_k),
                    phase: component_phase(d)?,
                })
            })
            .collect::<Result<Vec<_>, SimError>>()?,
    )
}

/// Parse a component's `phase = "..."` field. Absent is liquid.
fn component_phase(def: &ComponentDef) -> Result<Phase, SimError> {
    match def.phase.as_deref() {
        None | Some("liquid") => Ok(Phase::Liquid),
        Some("gas") => Ok(Phase::Gas),
        Some(other) => Err(SimError::Scenario(format!(
            "component '{}' has unknown phase '{other}' (valid: liquid, gas)",
            def.name
        ))),
    }
}

/// Resolve a node's `composition = { name = weight, ... }` against the slate.
///
/// Weights are normalized (`Composition::from_weights`), so a file may write
/// fractions summing to 1 or raw mass amounts — whichever reads better — and
/// both mean the same thing. Unknown names are rejected rather than ignored: a
/// typo'd cut name would otherwise silently drop that fraction and renormalize
/// the rest, producing a plausible-looking wrong feed.
///
/// An absent composition is only meaningful on a one-component slate, where
/// there is exactly one thing the fluid can be. On a real slate it is refused
/// rather than defaulted — see `NodeDef::Source::composition`.
fn resolve_composition(
    node: &str,
    weights: &Option<BTreeMap<String, f64>>,
    slate: &Slate,
) -> Result<Composition, SimError> {
    let Some(weights) = weights else {
        if slate.len() == 1 {
            return Ok(Composition::pure(1, 0));
        }
        return Err(SimError::Scenario(format!(
            "node '{node}' has no composition, but the slate has {} components. \
             Only a one-component slate has an unambiguous default; write \
             composition = {{ <component> = <weight>, ... }}",
            slate.len()
        )));
    };
    let mut fractions = vec![0.0; slate.len()];
    for (name, weight) in weights {
        let index = slate.index_of(name).ok_or_else(|| {
            SimError::Scenario(format!(
                "node '{node}' references unknown component '{name}'; the slate \
                 defines: {}",
                slate
                    .iter()
                    .map(|c| c.name.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            ))
        })?;
        fractions[index] = *weight;
    }
    Composition::from_weights(&fractions)
        .map_err(|e| SimError::Scenario(format!("node '{node}': {e}")))
}

/// Convert a scenario node definition into a core `NodeKind`, applying the
/// human-friendly → SI conversions at this boundary (bar → Pa, °C → K,
/// metric Kv → element cv_si, tank level → mass).
///
/// A tank's initial mass is `ρ·A·h` at the density of ITS OWN contents, not at
/// water's. With a one-component slate the two coincide, which is why this went
/// unnoticed through M1 and M2; with a crude slate, filling a tank with a light
/// cut and computing its mass at 998 kg/m³ would over-charge the inventory by
/// ~40% and break mass conservation at tick zero.
fn node_kind(name: &str, def: &NodeDef, slate: &Slate) -> Result<NodeKind, SimError> {
    Ok(match def {
        NodeDef::Source {
            pressure_bar,
            temperature_c,
            composition,
        } => NodeKind::Source {
            pressure: bar_to_pa(*pressure_bar),
            temperature: c_to_k(*temperature_c),
            composition: resolve_composition(name, composition, slate)?,
        },
        NodeDef::Sink {
            pressure_bar,
            temperature_c,
            composition,
        } => NodeKind::Sink {
            pressure: bar_to_pa(*pressure_bar),
            temperature: c_to_k(*temperature_c),
            // Through the same resolver a source uses, so an absent composition
            // on a real slate is REFUSED rather than defaulted. This is the one
            // place a sink's composition does not mirror its temperature, and
            // the asymmetry is in the physics, not the design: ambient is a
            // defensible neutral temperature to back-feed, and there is no
            // corresponding neutral composition — "the first cut" is a guess
            // that would run.
            composition: resolve_composition(name, composition, slate)?,
        },
        NodeDef::Atmosphere => NodeKind::Atmosphere,
        NodeDef::Tank {
            area_m2,
            height_m,
            initial_level_m,
            temperature_c,
            ambient_ua_w_per_k,
            composition,
        } => {
            let area = SquareMeter(*area_m2);
            let composition = resolve_composition(name, composition, slate)?;
            // A tank holds a LIQUID, and the two lines below are why the guard
            // is here rather than left to the connected-component check: both
            // `ρ·A·h` and `bottom_pressure`'s `ρgh` read a stored liquid density,
            // and on a gas composition there is none — an inventory and a
            // hydrostatic head computed from a level are not merely inaccurate
            // for a gas, they name nothing. A gas holdup is the capacitive
            // vessel (M5.3), whose state is pressure, not level.
            if composition.phase(slate)? == Phase::Gas {
                return Err(SimError::Scenario(format!(
                    "tank '{name}' holds a gas-phase composition. A tank's inventory \
                     (ρ·A·h) and head (ρgh) are liquid-level quantities; a gas holdup \
                     is a capacitive vessel, whose state is pressure (docs/DESIGN.md §3a)"
                )));
            }
            // m = ρ·A·h, at the density of the tank's own contents.
            let density = composition.mixture_density(slate);
            let mass = Kg(density.value() * area.value() * initial_level_m);
            NodeKind::Tank(TankState {
                area,
                height: Meter(*height_m),
                mass,
                temperature: c_to_k(*temperature_c),
                composition,
                ambient_ua: WattPerKelvin(*ambient_ua_w_per_k),
            })
        }
        NodeDef::Vessel {
            volume_m3,
            pressure_bar,
            temperature_c,
            composition,
        } => {
            let composition = resolve_composition(name, composition, slate)?;
            // The mirror of the tank's liquid-only guard, and the other half of
            // the same partition: a holdup is a tank if its state is a level and
            // a vessel if its state is a pressure. `C = V·M̄/(R·T)` is the
            // ideal-gas relation — for an incompressible liquid it is not merely
            // inaccurate, it names nothing, and it would silently produce a
            // capacitance ~5 orders too small and a plant that oscillates.
            if composition.phase(slate)? != Phase::Gas {
                return Err(SimError::Scenario(format!(
                    "vessel '{name}' holds a liquid-phase composition. A vessel's state is \
                     PRESSURE and its capacitance C = V·M̄/(R·T) is the ideal-gas relation; \
                     a liquid holdup is a tank, whose state is level (docs/DESIGN.md §3a)"
                )));
            }
            let mut vessel = VesselState {
                volume: CubicMeter(*volume_m3),
                // Filled in immediately below, from the capacitance this same
                // struct computes. Going through `capacitance` rather than
                // writing `P·V·M̄/(R·T)` out again is what guarantees the vessel's
                // `pressure()` reads back EXACTLY the declared bar figure — and
                // therefore that the accumulation term's `Pⁿ` starts where the
                // file says the plant does.
                mass: Kg(0.0),
                temperature: c_to_k(*temperature_c),
                composition,
            };
            vessel.mass = Kg(bar_to_pa(*pressure_bar).value() * vessel.capacitance(slate));
            NodeKind::Vessel(vessel)
        }
        NodeDef::Pump { h0_m, a, on } => NodeKind::Pump {
            h0: Meter(*h0_m),
            a: *a,
            on: *on,
        },
        // `x_t` is carried through unvalidated HERE and checked in
        // `require_gas_valve_x_t` instead: whether a valve is in gas service is a
        // TOPOLOGICAL fact, and the topology does not exist yet at this point in
        // the load.
        NodeDef::Valve { kv, opening, x_t } => NodeKind::Valve {
            cv_max: kv_to_cv_si(*kv),
            opening: *opening,
            x_t: *x_t,
        },
        NodeDef::ReliefValve {
            kv,
            set_pressure_bar,
            accumulation_bar,
            x_t,
        } => {
            if !accumulation_bar.is_finite() || *accumulation_bar <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "relief valve '{name}' has accumulation_bar = {accumulation_bar}, which                      must be > 0. A zero band makes the opening a STEP in pressure, and a                      discontinuous characteristic is exactly what elements.rs promises not                      to hand the Newton Jacobian."
                )));
            }
            if !set_pressure_bar.is_finite() || *set_pressure_bar <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "relief valve '{name}' has set_pressure_bar = {set_pressure_bar}; a set                      pressure is ABSOLUTE and must be positive."
                )));
            }
            NodeKind::ReliefValve {
                cv_max: kv_to_cv_si(*kv),
                set_pressure: bar_to_pa(*set_pressure_bar),
                accumulation: bar_to_pa(*accumulation_bar),
                x_t: *x_t,
            }
        }
        NodeDef::Furnace { duty_mw } => NodeKind::Furnace {
            duty: Watt(*duty_mw * 1e6),
        },
        NodeDef::Cooler { duty_mw } => NodeKind::Cooler {
            duty: Watt(*duty_mw * 1e6),
        },
        NodeDef::HeatExchanger => NodeKind::HeatExchanger,
        NodeDef::Junction => NodeKind::Junction,
        NodeDef::Column {
            pressure_bar,
            smearing_k,
            ..
        } => NodeKind::Column {
            pressure: bar_to_pa(*pressure_bar),
            // A temperature WIDTH, so no °C→K offset — a 10 °C ramp is 10 K.
            smearing: Kelvin(*smearing_k),
            // Draws are resolved in a second pass, once every node exists so a
            // draw can name an outlet defined later in the file (like couplings).
            draws: Vec::new(),
        },
        NodeDef::Reactor { t_set_c, tau_s } => NodeKind::Reactor {
            t_set: c_to_k(*t_set_c),
            tau: Seconds(*tau_s),
        },
    })
}

/// Reject node definitions whose numbers are out of physical range, at load
/// rather than at solve.
///
/// Duty is a non-negative MAGNITUDE in both units — direction is the unit's
/// identity, not the sign of its number (see `NodeKind::Cooler`). A negative
/// `furnace` duty used to be the only way to express cooling; now that `cooler`
/// exists it can only be a sign slip, and a silently-chilling furnace is a
/// plausible-looking wrong plant. Rejecting it here is what makes the two-unit
/// design safe rather than merely tidy.
///
/// A tank's `UA` is refused on the same grounds. It is a CONDUCTANCE, not a
/// signed rate: the direction of ambient exchange already comes from
/// `T_AMBIENT − T_tank`, so a negative `UA` does not mean "loses heat" — it
/// inverts the driving force, warming a hot tank further and cooling a cold one,
/// a positive feedback that runs away from ambient instead of towards it.
/// Reject a pipe's `UA` on the same grounds as a tank's, and one sharper one.
///
/// For a tank a negative `UA` inverts the driving force — bad, but the runaway
/// is geometric per tick and bounded by the tick count. For a pipe it lands in
/// an EXPONENT: `exp(−UA/(|ṁ|·cp))` with `UA < 0` is `exp(+x)`, which multiplies
/// the temperature difference from ambient every time the fluid crosses the
/// pipe, and a loop of such pipes diverges to infinity within a few ticks. Same
/// conceptual error, a much shorter fuse.
/// Build a declared pipe as a LEAK PATH: two halves joined by a `Junction`, with
/// a dormant orifice edge from that junction to `atmosphere`.
///
/// **Split at LOAD, not at puncture** (docs/DESIGN.md §3b). Splitting when the
/// damage happens would change the snapshot's shape mid-run, which is a rule-6
/// contract problem for every frontend; splitting at load fixes the topology
/// before tick 0, so a punctured plant and an intact one have the same shape and
/// differ only in one commanded number.
///
/// **Hanging the orifice off an ENDPOINT was not merely inelegant, it was
/// illegal.** `validate_degrees` requires exactly 1-in-1-out of a pump, valve,
/// PSV, furnace, cooler, reactor and exchanger side, so a third edge leaving any
/// of them is a load-time `Err` — and a pipe's upstream endpoint is a pump or a
/// valve constantly (`tank_pump_valve.toml` is nothing but). An endpoint rule
/// would therefore have forbidden leaks on exactly the lines a game most wants
/// to puncture. The midpoint junction has no degree rule on it, and it carries
/// the physically right pressure for a mid-pipe hole into the bargain: neither
/// endpoint's, but the one between them.
///
/// The halves split the length, the elevation and the `UA` evenly and keep the
/// diameter and friction factor, so the two in series are hydraulically the
/// declared pipe: `k ∝ L` adds back to the original, `β = ρ·g·Δz` adds back, and
/// the ambient transform composes over the two halves. **The upstream half keeps
/// the declared NAME** — it is what `PuncturePipe` addresses and where
/// `leak_mass_flow` is reported.
fn split_for_leak(
    graph: &mut PlantGraph,
    def: &PipeDef,
    from: NodeId,
    to: NodeId,
    whole: Pipe,
    atmosphere: &str,
) -> Result<(), SimError> {
    let vent = graph.find_node(atmosphere).ok_or_else(|| {
        SimError::Scenario(format!(
            "pipe '{}' declares leak_to = '{atmosphere}', which is not a node in this plant",
            def.name
        ))
    })?;
    if !matches!(graph.node(vent).kind, NodeKind::Atmosphere) {
        return Err(SimError::Scenario(format!(
            "pipe '{}' declares leak_to = '{atmosphere}', which is a {:?}, not an atmosphere. \
             A leak vents to the outside world; venting it into the plant would be an \
             ordinary pipe, and the scenario should say so",
            def.name,
            graph.node(vent).kind
        )));
    }
    // A COLUMN's pipes cannot be split, and this refusal is here because the
    // failure it prevents is silent. `network::is_column_draw_edge` recognises a
    // draw by its two endpoints — column at one end, one of that column's
    // declared outlets at the other — and `edge_flows` guards a draw's flow to
    // zero on the strength of it, because a draw's flow is PRESCRIBED
    // (`splitᵢ·ṁ_feed`, written post-sweep) and not pressure-driven at all.
    // Split that edge and neither half matches any more, so the guard silently
    // stops applying and the draw becomes a pressure-driven number that is
    // finite, deterministic, mass-conserving and wrong — DESIGN §5's silent
    // hazard, reached by a scenario line that looks entirely reasonable. The feed
    // is refused with it: a column is 1-in-N-out by `validate_degrees`, so a
    // split feed would fail there anyway, but with a message about degrees that
    // names the wrong cause.
    for end in [from, to] {
        if matches!(graph.node(end).kind, NodeKind::Column { .. }) {
            return Err(SimError::Scenario(format!(
                "pipe '{}' declares a leak path but connects to column '{}'. A column's \
                 feed and draw pipes cannot be split: a draw's flow is prescribed by the \
                 feed split, not by pressure, and splitting it would silently turn it \
                 into a pressure-driven flow (docs/DESIGN.md §3b, §5)",
                def.name,
                graph.node(end).name
            )));
        }
    }

    let junction = format!("{}__leak_point", def.name);
    if graph.find_node(&junction).is_some() {
        return Err(SimError::Scenario(format!(
            "pipe '{}' declares a leak path, whose midpoint junction would be named \
             '{junction}' — and this plant already has a node by that name. Rename one",
            def.name
        )));
    }
    let mid = graph.add_node(Node {
        name: junction,
        kind: NodeKind::Junction,
        heat_input: Watt::ZERO,
    });

    // Half a pipe each: k ∝ L and β = ρ·g·Δz both add back to the declared pipe,
    // and UA ∝ exposed area does too.
    let half = |name: String| Pipe {
        name,
        length: Meter(def.length_m / 2.0),
        elevation_change: Meter(def.elevation_change_m / 2.0),
        ambient_ua: WattPerKelvin(def.ambient_ua_w_per_k / 2.0),
        ..whole.clone()
    };
    let upstream = graph.add_pipe(from, mid, half(def.name.clone()));
    graph.add_pipe(mid, to, half(format!("{}__downstream", def.name)));

    // The orifice runs junction → atmosphere, so positive graph direction is
    // OUTWARD and `leak_mass_flow` needs no sign flip. Its geometry is ZERO on
    // purpose: an orifice has no length to resist with and no bore that means
    // anything (its area is commanded), and `compile_edge` returns before reading
    // either. Should that early return ever be removed, `pipe_resistance` on a
    // zero length and a zero diameter is non-finite and the edge fails loudly at
    // the first solve — which is the point of writing zeros rather than plausible
    // numbers that would quietly become a second resistance in the leak path.
    let orifice = graph.add_pipe(
        mid,
        vent,
        Pipe {
            name: format!("{}__leak", def.name),
            length: Meter(0.0),
            diameter: Meter(0.0),
            elevation_change: Meter(0.0),
            ambient_ua: WattPerKelvin(0.0),
            leak: LeakRole::Orifice {
                area: SquareMeter::ZERO,
            },
            ..whole
        },
    );
    graph.pipe_mut(upstream).leak = LeakRole::Punctureable { orifice };
    Ok(())
}

fn validate_pipe_def(def: &PipeDef) -> Result<(), SimError> {
    if !def.ambient_ua_w_per_k.is_finite() || def.ambient_ua_w_per_k < 0.0 {
        return Err(SimError::Scenario(format!(
            "pipe '{}' has ambient_ua_w_per_k = {}: it must be finite and >= 0. UA \
             is a conductance; the DIRECTION of ambient exchange comes from \
             (T_ambient − T_in), so a negative value does not mean 'loses heat' — \
             in a pipe it inverts the exponent and drives the outlet away from \
             ambient without bound.",
            def.name, def.ambient_ua_w_per_k
        )));
    }
    Ok(())
}

fn validate_node_def(name: &str, def: &NodeDef) -> Result<(), SimError> {
    if let NodeDef::Tank {
        ambient_ua_w_per_k, ..
    } = def
    {
        if !ambient_ua_w_per_k.is_finite() || *ambient_ua_w_per_k < 0.0 {
            return Err(SimError::Scenario(format!(
                "tank '{name}' has ambient_ua_w_per_k = {ambient_ua_w_per_k}: it must \
                 be finite and >= 0. UA is a conductance; the DIRECTION of ambient \
                 exchange comes from (T_ambient − T_tank), so a negative value does \
                 not mean 'loses heat' — it drives the tank away from ambient."
            )));
        }
    }
    if let NodeDef::Vessel {
        volume_m3,
        pressure_bar,
        temperature_c,
        ..
    } = def
    {
        // Every one of these three sits in `C = V·M̄/(R·T)` or in `m = C·P`, and
        // a non-positive value in any of them produces a finite, plausible-looking
        // capacitance or inventory rather than an obvious failure: V ≤ 0 gives a
        // vessel that stores nothing (the zero-volume junction it is not), T ≤ 0
        // flips or blows up C, and P ≤ 0 starts the vessel at or below vacuum with
        // no gas in it. Caught at load, where the file can be named.
        if !volume_m3.is_finite() || *volume_m3 <= 0.0 {
            return Err(SimError::Scenario(format!(
                "vessel '{name}' has volume_m3 = {volume_m3}: it must be finite and > 0 \
                 (a vessel with no volume has no capacitance, which is a junction)."
            )));
        }
        if !pressure_bar.is_finite() || *pressure_bar <= 0.0 {
            return Err(SimError::Scenario(format!(
                "vessel '{name}' has pressure_bar = {pressure_bar}: it must be finite and > 0. \
                 It is the vessel's STATE (its mass follows from it), not a setpoint."
            )));
        }
        if !temperature_c.is_finite() || c_to_k(*temperature_c).value() <= 0.0 {
            return Err(SimError::Scenario(format!(
                "vessel '{name}' has temperature_c = {temperature_c}: it must be finite and \
                 above absolute zero (−273.15 °C); it divides into the capacitance."
            )));
        }
    }
    if let NodeDef::Column {
        pressure_bar,
        smearing_k,
        ..
    } = def
    {
        if !pressure_bar.is_finite() || *pressure_bar <= 0.0 {
            return Err(SimError::Scenario(format!(
                "column '{name}' has pressure_bar = {pressure_bar}: it must be finite and > 0 \
                 (a pinned operating pressure)."
            )));
        }
        // Zero is legal — the sharp splitter. Negative is not: a ramp width is a
        // magnitude, and `smearing < 0` would flip the ramp's slope.
        if !smearing_k.is_finite() || *smearing_k < 0.0 {
            return Err(SimError::Scenario(format!(
                "column '{name}' has smearing_k = {smearing_k}: it must be finite and >= 0 \
                 (a ramp width; 0 is a sharp splitter)."
            )));
        }
    }
    if let NodeDef::Reactor { t_set_c, tau_s } = def {
        // A residence time is a non-negative magnitude; a negative one is a sign
        // slip, and the M4.2 kinetics would integrate over a backwards interval.
        if !tau_s.is_finite() || *tau_s < 0.0 {
            return Err(SimError::Scenario(format!(
                "reactor '{name}' has tau_s = {tau_s}: residence time must be finite and >= 0."
            )));
        }
        // The setpoint is an absolute temperature; below 0 K it is unphysical and
        // would seed the sweep with a negative outlet the datum cannot represent.
        if !t_set_c.is_finite() || c_to_k(*t_set_c).value() < 0.0 {
            return Err(SimError::Scenario(format!(
                "reactor '{name}' has t_set_c = {t_set_c}: the setpoint must be finite and \
                 at or above absolute zero (−273.15 °C)."
            )));
        }
    }
    let duty = match def {
        NodeDef::Furnace { duty_mw } => Some(("furnace", *duty_mw)),
        NodeDef::Cooler { duty_mw } => Some(("cooler", *duty_mw)),
        _ => None,
    };
    if let Some((unit, duty_mw)) = duty {
        if !duty_mw.is_finite() || duty_mw < 0.0 {
            return Err(SimError::Scenario(format!(
                "{unit} '{name}' has duty_mw = {duty_mw}: duty must be finite and \
                 >= 0. It is a magnitude — to remove heat use a 'cooler' node, \
                 not a negative furnace duty."
            )));
        }
    }
    Ok(())
}

/// Resolve the `[[exchangers]]` table into graph couplings, rejecting every way
/// a pairing can be malformed.
///
/// The checks are not defensive noise — each rules out a plant that would
/// otherwise RUN and report plausible temperatures:
///
/// - **ε outside (0, 1]** transfers more heat than the inlet temperature
///   difference makes available, crossing the outlets. That is a second-law
///   violation the sweep cannot detect locally, since every intermediate number
///   stays finite and positive.
/// - **A side paired twice** would give one node two partners, and the energy
///   sweep's pair merge silently uses whichever coupling it finds first.
/// - **A side paired with itself** makes the exchanger its own upstream.
/// - **An uncoupled `heat_exchanger` node** is not an exchanger at all — it
///   would behave as a plain junction, transferring nothing, which is exactly
///   what a scenario author who forgot the table would fail to notice.
fn build_couplings(graph: &mut PlantGraph, defs: &[ExchangerDef]) -> Result<(), SimError> {
    let mut paired: BTreeMap<NodeId, String> = BTreeMap::new();

    for def in defs {
        if !(def.effectiveness.is_finite() && def.effectiveness > 0.0 && def.effectiveness <= 1.0) {
            return Err(SimError::Scenario(format!(
                "exchanger '{}'/'{}' has effectiveness = {}: it must lie in (0, 1]. \
                 Above 1 the exchanger would transfer more than the inlet \
                 temperature difference allows and cross the outlet temperatures; \
                 0 or less is not an exchanger.",
                def.side_a, def.side_b, def.effectiveness
            )));
        }
        if def.side_a == def.side_b {
            return Err(SimError::Scenario(format!(
                "exchanger pairs '{}' with itself: the two sides must be \
                 different nodes",
                def.side_a
            )));
        }

        let resolve = |name: &str| -> Result<NodeId, SimError> {
            let id = graph.find_node(name).ok_or_else(|| {
                SimError::Scenario(format!("exchanger references unknown node '{name}'"))
            })?;
            if !matches!(graph.node(id).kind, NodeKind::HeatExchanger) {
                return Err(SimError::Scenario(format!(
                    "exchanger side '{name}' is a {:?}, not a heat_exchanger node",
                    graph.node(id).kind
                )));
            }
            Ok(id)
        };
        let side_a = resolve(&def.side_a)?;
        let side_b = resolve(&def.side_b)?;

        for (id, name) in [(side_a, &def.side_a), (side_b, &def.side_b)] {
            if let Some(other) = paired.get(&id) {
                return Err(SimError::Scenario(format!(
                    "exchanger side '{name}' is paired more than once (already \
                     coupled with '{other}'): each side has exactly one partner"
                )));
            }
            paired.insert(id, name.clone());
        }

        graph.add_coupling(HeatExchangerCoupling {
            side_a,
            side_b,
            effectiveness: def.effectiveness,
        });
    }

    // Every side must be in the table: an unpaired one is a silent plain pipe.
    for id in graph.node_ids().collect::<Vec<_>>() {
        if matches!(graph.node(id).kind, NodeKind::HeatExchanger) && !paired.contains_key(&id) {
            return Err(SimError::Scenario(format!(
                "heat_exchanger node '{}' is not paired in any [[exchangers]] \
                 entry: an unpaired side transfers no heat at all and would run \
                 as a plain junction",
                graph.node(id).name
            )));
        }
    }
    Ok(())
}

/// Resolve every `column` node's draws (name → NodeId) and reject every way a
/// draw list can be malformed. Runs after all nodes exist so a draw may name an
/// outlet defined later in the file, exactly like an exchanger coupling.
///
/// Each check rules out a plant that would otherwise load and be wrong:
///
/// - **Fewer than two draws** separates nothing — use a pipe.
/// - **The catch-all convention** — exactly the LAST draw omits `up_to_c`. The
///   heaviest draw is open-topped so every component lands somewhere; if it had a
///   finite top, components above it would be split into no draw and lost, a
///   silent mass leak. Making "which draw is the residue" a syntactic fact keeps
///   conservation a load-time guarantee.
/// - **Cut points strictly increasing** — a flat or inverted cut is an empty or
///   backwards band.
/// - **Distinct outlets** — the engine maps a draw edge to a draw by its outlet,
///   so the mapping has to be one-to-one.
/// - **Outlet is a product store** (tank/sink/atmosphere) — a free node on a draw
///   line puts a prescribed edge back into the Jacobian (unsupported), and a
///   source or column outlet is pressure-fixing but still wrong (vanishes the
///   product, or chains columns the draw write cannot feed); both refused.
fn resolve_column_draws(graph: &mut PlantGraph, scenario: &ScenarioFile) -> Result<(), SimError> {
    for (name, def) in &scenario.nodes {
        let NodeDef::Column {
            draws: draw_defs, ..
        } = def
        else {
            continue;
        };
        if draw_defs.len() < 2 {
            return Err(SimError::Scenario(format!(
                "column '{name}' has {} draw(s): a column needs at least two, else it \
                 separates nothing (use a pipe).",
                draw_defs.len()
            )));
        }

        let last = draw_defs.len() - 1;
        let mut prev_cut = f64::NEG_INFINITY;
        let mut resolved: Vec<ColumnDraw> = Vec::with_capacity(draw_defs.len());
        for (i, d) in draw_defs.iter().enumerate() {
            // Exactly the last draw omits up_to_c — the open catch-all.
            let upper_cut_c = match (i == last, d.up_to_c) {
                (true, None) => None,
                (false, Some(c)) => Some(c),
                (true, Some(_)) => {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draw '{}' is the heaviest (last) draw and must OMIT \
                         up_to_c: it is the catch-all for everything above the last cut, so a \
                         finite top would drop every heavier component.",
                        d.outlet
                    )))
                }
                (false, None) => {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draw '{}' omits up_to_c but is not the last draw: only \
                         the heaviest (last) draw may — every other draw needs a boiling-range top.",
                        d.outlet
                    )))
                }
            };
            if let Some(c) = upper_cut_c {
                let cut_k = c_to_k(c).value();
                if cut_k <= prev_cut {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draw '{}' has up_to_c = {c} °C, not strictly above the \
                         previous cut: draws must be listed in ascending boiling order.",
                        d.outlet
                    )));
                }
                prev_cut = cut_k;
            }

            if draw_defs[..i].iter().any(|e| e.outlet == d.outlet) {
                return Err(SimError::Scenario(format!(
                    "column '{name}' draws to '{}' more than once: each draw feeds a distinct \
                     product node.",
                    d.outlet
                )));
            }

            let outlet = graph.find_node(&d.outlet).ok_or_else(|| {
                SimError::Scenario(format!(
                    "column '{name}' draw references unknown outlet node '{}'",
                    d.outlet
                ))
            })?;
            // A draw must end at a PRODUCT STORE. "Pressure-fixing" is necessary
            // (a free node would put a prescribed edge in the Jacobian) but NOT
            // sufficient: `fixed_pressure` is also `Some` for a Source and a
            // Column, and both are silently wrong outlets. Drawing to a Source
            // would vanish the product into an infinite supply — mass "conserved"
            // at the boundary, a plant that runs and lies. Drawing to another
            // Column would chain them, and the post-sweep two-pass draw write
            // reads the upstream draw edge before the downstream column's write
            // lands, so the second column silently sees a zero feed and does
            // nothing. Neither is modelled at this fidelity, so the outlet is
            // restricted to the three product-store kinds explicitly.
            match &graph.node(outlet).kind {
                NodeKind::Tank(_) | NodeKind::Sink { .. } | NodeKind::Atmosphere => {}
                NodeKind::Source { .. } | NodeKind::Column { .. } => {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draws to '{}', a source or column. A draw must end at a \
                         product store — a tank, sink, or atmosphere. Drawing to a source vanishes \
                         the product into a supply, and chaining columns is not supported at this \
                         fidelity.",
                        d.outlet
                    )));
                }
                _ => {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draws to '{}', a free (non-pressure-fixing) node. A draw \
                         line must end at a product store (tank/sink/atmosphere); a valve or \
                         junction on a draw is not supported at this fidelity.",
                        d.outlet
                    )));
                }
            }
            resolved.push(ColumnDraw {
                outlet,
                upper_cut: upper_cut_c.map(c_to_k),
            });
        }

        let col_id = graph.find_node(name).expect("column node was just added");
        if let NodeKind::Column { draws, .. } = &mut graph.node_mut(col_id).kind {
            *draws = resolved;
        }
    }
    Ok(())
}

fn bar_to_pa(bar: f64) -> Pascal {
    Pascal(bar * 1e5)
}
fn c_to_k(celsius: f64) -> Kelvin {
    Kelvin(celsius + 273.15)
}
/// Metric Kv [m³/h at ΔP = 1 bar, SG = 1] → element cv_si used by
/// `valve_flow`: Q[m³/s] = cv_si·opening·√(dP_Pa/ρ_rel). Starting from the
/// Kv definition Q[m³/h] = Kv·√(dP_bar/SG), convert h→s (÷3600) and bar→Pa
/// inside the root (÷√1e5): cv_si = Kv / (3600·√1e5).
fn kv_to_cv_si(kv: f64) -> f64 {
    kv / (3600.0 * 1e5_f64.sqrt())
}

/// Validate the built graph at load time so a structural fault fails here with
/// a clear message rather than as a divergent solve later.
fn validate_topology(graph: &PlantGraph, slate: &Slate) -> Result<(), SimError> {
    // (a) Pump/valve (1-inlet, 1-outlet) degree. Reuse the solver's canonical
    //     invariant — single owner, no drift — remapped to a load-time error.
    refinery_solvers::network::validate_degrees(graph)
        .map_err(|e| SimError::Scenario(e.to_string()))?;

    // (b) Every connected component must contain at least one pressure-fixing
    //     node (Source/Sink/Atmosphere/Tank), else its hydraulic problem is
    //     singular. Connectivity walks ALL pipes — NOT the solver's
    //     conducting-edge anchoring — so a valve closed at t=0 cannot falsely
    //     sever the network at load. Union-Find keyed on NodeId.0 (dense 0..n
    //     because the graph is freshly built with no removals).
    let ids: Vec<NodeId> = graph.node_ids().collect();
    let mut parent: Vec<usize> = (0..ids.len()).collect();
    for eid in graph.edge_ids() {
        let (a, b) = graph.endpoints(eid);
        uf_union(&mut parent, a.0 as usize, b.0 as usize);
    }
    // Per component root: whether it has a pressure fixer, and its lowest-id
    // member (deterministic representative for the error message).
    let mut has_fixer: BTreeMap<usize, bool> = BTreeMap::new();
    let mut representative: BTreeMap<usize, NodeId> = BTreeMap::new();
    for &id in &ids {
        let root = uf_find(&mut parent, id.0 as usize);
        let fixes = provides_pressure_reference(graph.node(id), slate);
        *has_fixer.entry(root).or_insert(false) |= fixes;
        representative.entry(root).or_insert(id);
    }
    for (root, fixed) in &has_fixer {
        if !*fixed {
            let member = representative[root];
            return Err(SimError::Scenario(format!(
                "network component containing '{}' has no pressure reference \
                 (needs at least one source/sink/atmosphere/tank, or a capacitive vessel); its hydraulic \
                 problem is singular",
                graph.node(member).name
            )));
        }
    }

    // (c) Column draw wiring. `validate_degrees` above pins the COUNTS (1 feed
    //     in, N draws out by graph direction); this pins the TARGETS, so a draw's
    //     splitᵢ·ṁ_feed is routed to the product the file named and is never
    //     silently dropped or doubled. An outgoing edge to a non-draw node, or a
    //     draw with zero or two outlet pipes, is a wiring error caught here.
    for nid in graph.node_ids() {
        let NodeKind::Column { draws, .. } = &graph.node(nid).kind else {
            continue;
        };
        let mut outlet_hits: BTreeMap<NodeId, usize> =
            draws.iter().map(|d| (d.outlet, 0usize)).collect();
        for (eid, other, incoming) in graph.incident(nid) {
            if incoming {
                continue; // the feed edge; its single-ness is a degree check
            }
            match outlet_hits.get_mut(&other) {
                Some(count) => *count += 1,
                None => {
                    return Err(SimError::Scenario(format!(
                        "column '{}' has an outlet pipe '{}' to '{}', which is not one of its \
                         draws: every pipe leaving a column must be a declared draw.",
                        graph.node(nid).name,
                        graph.pipe(eid).name,
                        graph.node(other).name
                    )))
                }
            }
        }
        for (outlet, count) in &outlet_hits {
            if *count != 1 {
                return Err(SimError::Scenario(format!(
                    "column '{}' draw to '{}' is fed by {count} pipe(s); it needs exactly one \
                     (a draw carries splitᵢ·ṁ_feed and must have a single outlet edge).",
                    graph.node(nid).name,
                    graph.node(*outlet).name
                )));
            }
        }
    }
    Ok(())
}

/// The phase of every node's connected component — the load-time guard that
/// makes M5's two-phase deferral loud instead of silent (docs/DESIGN.md §3a).
///
/// A model with no phase equilibrium must never be handed a two-phase mixture,
/// because it would volume-average it into a fluid that is neither. Refusing
/// two-phase *compositions* is not enough on its own: streams mix at runtime
/// wherever two lines join, so a gas source and a liquid source that share a
/// junction would produce one by blending. The check is therefore TOPOLOGICAL —
/// every connected component of the plant graph is all-gas or all-liquid.
///
/// Connectivity walks ALL pipes, exactly like the pressure-reference check, so a
/// valve closed at t=0 cannot make an illegal plant legal by severing it.
///
/// Nodes that declare no composition (`Atmosphere`, junctions, valves, pumps,
/// furnaces, coolers, exchanger sides, columns, reactors) do not vote: they
/// carry whatever reaches them. A component in which nothing votes has no phase,
/// and `Liquid` is then the answer that changes nothing.
///
/// Every valve in GAS service must declare `x_t`, and every valve in LIQUID
/// service must not (docs/DESIGN.md §3a forks 4 and 6).
///
/// **The definition of "gas service" is `plant_phases`, reused — not a second
/// test of the same thing.** `cv_si` is one field for both services, so something
/// has to decide when the compressible law applies; building an independent
/// notion here would give two answers that can disagree, and the failure mode is
/// a plant that loads with no `x_T` and silently runs the liquid branch on gas.
/// That is why this runs after `plant_phases` and takes its verdict verbatim.
///
/// Enforced in BOTH directions, exactly as `PseudoComponent::density` is: a
/// liquid valve carrying an `x_t` is refused, because a number nothing reads is
/// how an author comes to believe the model uses something it does not — the
/// argument that kept a "gas Cv" out of this milestone in the first place.
///
/// No default, deliberately. `x_T` is the one genuinely new coefficient here, and
/// a silent default would be an invented value in disguise: the sizing gate and
/// the choked-plateau gate would both pass for whatever it was, which is the
/// circularity that defers pump `η`.
/// Refuse a leak path declared on a gas line, naming the file that declared it.
///
/// The orifice law this milestone ships is Torricelli, `Q = Cd·A·√(2·dp/ρ)`,
/// which is the incompressible one. A hole venting a pressurised gas line to
/// atmosphere is choked over essentially its entire useful range — the critical
/// ratio is ~0.53 of absolute inlet pressure for a diatomic gas, so anything
/// above ~1.9 bara chokes — and applying the incompressible law there
/// overpredicts the escape rate: finite, deterministic and wrong, in the one
/// number a damage model exists to report. It is refused rather than
/// approximated for exactly the reason M5.4 refused an incompressible gas VALVE.
///
/// This is the FIRST of two doors. `network::compile_edge` re-refuses it, because
/// the loader is not the only way in: the invariant proptests build a
/// `PlantGraph` directly and never call `build_engine`. This one exists to name
/// the scenario file; that one exists to catch a generator.
///
/// It un-defers with an orifice `x_T` and a published anchor to size it against.
fn refuse_gas_leak(graph: &PlantGraph, phases: &[Phase]) -> Result<(), SimError> {
    for eid in graph.edge_ids() {
        let pipe = graph.pipe(eid);
        if !matches!(pipe.leak, LeakRole::Orifice { .. }) {
            continue;
        }
        // The orifice's SOURCE is the midpoint junction, i.e. the plant side —
        // the phase of the line being punctured, which is what the law has to
        // suit. Its target is the Atmosphere, which declares no composition and
        // votes on nothing.
        let (plant_side, _) = graph.endpoints(eid);
        if phases[plant_side.0 as usize] == Phase::Gas {
            return Err(SimError::Scenario(format!(
                "leak path '{}' is on a gas line. The orifice law is incompressible \
                 (Q = Cd·A·√(2·dp/ρ)), and a hole venting gas to atmosphere is choked \
                 over its whole useful range, so it would report a leak rate that is \
                 too high — the compressible-law-on-a-compressible-fluid refusal M5.4 \
                 made for valves (docs/DESIGN.md §3b)",
                pipe.name
            )));
        }
    }
    Ok(())
}

fn require_gas_valve_x_t(graph: &PlantGraph, phases: &[Phase]) -> Result<(), SimError> {
    for nid in graph.node_ids() {
        let node = graph.node(nid);
        // Both valve kinds, for one reason: they share `compile_edge`'s arm and
        // therefore the same compressible law, so they must share the same
        // requirement or a PSV could reach the gas branch with no `x_T`.
        let x_t = match &node.kind {
            NodeKind::Valve { x_t, .. } | NodeKind::ReliefValve { x_t, .. } => x_t,
            _ => continue,
        };
        match (phases[nid.0 as usize], x_t) {
            (Phase::Gas, None) => {
                return Err(SimError::Scenario(format!(
                    "valve '{}' is in gas service and must declare `x_t`, the IEC \
                     60534-2-1 pressure differential ratio factor. There is no default: \
                     it is per-valve manufacturer data, and a silent one would make the \
                     choke point an invented number. Typical values are tabulated by \
                     valve style in the standard.",
                    node.name
                )));
            }
            (Phase::Liquid, Some(x)) => {
                return Err(SimError::Scenario(format!(
                    "valve '{}' is in liquid service and declares `x_t = {x}`, which \
                     nothing reads: the expansion factor applies to compressible flow \
                     only. Remove it, or the file claims a model the engine does not run.",
                    node.name
                )));
            }
            _ => {}
        }
        if let Some(x) = x_t {
            if !(*x > 0.0 && *x < 1.0) {
                return Err(SimError::Scenario(format!(
                    "valve '{}' has x_t = {x}, outside (0, 1). The pressure differential \
                     ratio factor is a fraction of the inlet absolute pressure.",
                    node.name
                )));
            }
        }
    }
    Ok(())
}

/// Returns the phase per node, indexed by `NodeId.0` — the seed `build_engine`
/// needs for each pipe's initial stream.
fn plant_phases(graph: &PlantGraph, slate: &Slate) -> Result<Vec<Phase>, SimError> {
    let ids: Vec<NodeId> = graph.node_ids().collect();
    let mut parent: Vec<usize> = (0..ids.len()).collect();
    for eid in graph.edge_ids() {
        let (a, b) = graph.endpoints(eid);
        uf_union(&mut parent, a.0 as usize, b.0 as usize);
    }
    // Per component root: the phase voted so far, and the node that voted it —
    // so a conflict names BOTH offenders rather than one and "something else".
    let mut voted: BTreeMap<usize, (Phase, NodeId)> = BTreeMap::new();
    for &id in &ids {
        let Some(composition) = declared_composition(&graph.node(id).kind) else {
            continue;
        };
        // A single node's composition mixing phases is caught here, by the same
        // call, and reported against the node that declares it.
        let phase = composition
            .phase(slate)
            .map_err(|e| SimError::Scenario(format!("node '{}': {e}", graph.node(id).name)))?;
        let root = uf_find(&mut parent, id.0 as usize);
        match voted.get(&root) {
            None => {
                voted.insert(root, (phase, id));
            }
            Some((seen, by)) if *seen == phase => {}
            Some((seen, by)) => {
                return Err(SimError::Scenario(format!(
                    "nodes '{}' ({phase:?}) and '{}' ({seen:?}) are connected, so their \
                     streams can mix — but this model has no phase equilibrium and would \
                     volume-average the result into a fluid that is neither. Every \
                     connected component of the plant must be all-gas or all-liquid; a \
                     gas system is a separate sub-plant (docs/DESIGN.md §3a)",
                    graph.node(id).name,
                    graph.node(*by).name
                )))
            }
        }
    }
    Ok(ids
        .iter()
        .map(|&id| {
            let root = uf_find(&mut parent, id.0 as usize);
            voted.get(&root).map_or(Phase::Liquid, |(p, _)| *p)
        })
        .collect())
}

/// The composition a node DECLARES, if any. Only these vote on their
/// component's phase; everything else carries whatever reaches it.
fn declared_composition(kind: &NodeKind) -> Option<&Composition> {
    match kind {
        NodeKind::Source { composition, .. } | NodeKind::Sink { composition, .. } => {
            Some(composition)
        }
        NodeKind::Tank(t) => Some(&t.composition),
        // A vessel declares its contents like a tank does, so it votes on its
        // component's phase — and it is the node most likely to be the gas vote
        // in a mixed-slate file.
        NodeKind::Vessel(v) => Some(&v.composition),
        _ => None,
    }
}

/// The slate index a pipe's initial stream composition is seeded with, for a
/// pipe in an all-`phase` part of the plant.
///
/// Readers of the pipe's stored composition, grepped rather than assumed:
/// `compile_edge`'s transport density is the only one that reaches the SOLVE
/// (M3.1 moved `cp` to the resolved upwind node — see `energy::stream_cp_at`),
/// and `EdgeSnapshot` publishes the whole `Stream`, so it is also frontend-
/// visible state until transport overwrites it at the end of tick 1. Both make
/// the same demand of the seed: it must not be the WRONG PHASE, which on a mixed
/// slate is what `pure(0)` would give every gas line (a liquid density is ~150×
/// a gas one at 10 bar, so tick 1 would not be stale, it would be nonsense).
///
/// For an all-liquid slate the first liquid component IS index 0, so every
/// pre-M5.2 scenario keeps the seed it had — measured, not reasoned: the seven
/// repo scenarios run 200 ticks under both fidelities produce byte-identical
/// JSON against the pre-M5.2 build.
/// The fallback is index 0 for the same reason: a slate with no component of
/// the requested phase can only be one the requesting sub-plant never uses.
fn seed_component_index(slate: &Slate, phase: Phase) -> usize {
    slate
        .iter()
        .position(|c| c.phase == phase)
        .unwrap_or_default()
}

/// True if the node gives its connected component a pressure REFERENCE, so the
/// component's hydraulic problem is not singular.
///
/// Two ways to be one, and they are deliberately not the same predicate as
/// `network::fixed_pressure`:
///
/// - **Pinned** (Source/Sink/Atmosphere/Tank/Column) — the pressure is imposed.
/// - **Capacitive** (Vessel) — the pressure is an UNKNOWN, but the node supplies
///   its own equation for it (`C·(P − Pⁿ)/dt`), so the component is well posed
///   with no pinned node anywhere in it. This is the load-time half of
///   "capacitance is an anchor" (DESIGN §3a fork 2); without it a closed gas
///   system would be refused here before the solver ever got to demonstrate it.
///
/// Both arms delegate to the solver so load-time and solve-time cannot disagree
/// about what counts as a boundary.
fn provides_pressure_reference(node: &Node, slate: &Slate) -> bool {
    refinery_solvers::network::fixed_pressure(node, slate).is_some()
        || refinery_solvers::network::capacitance(node, slate).is_some()
}

fn uf_find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]]; // path halving
        x = parent[x];
    }
    x
}
fn uf_union(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (uf_find(parent, a), uf_find(parent, b));
    if ra != rb {
        parent[ra] = rb;
    }
}

#[cfg(test)]
mod tests {
    /// The reference scenario file must always parse against the schema.
    #[test]
    fn reference_scenario_parses() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).expect("tank_pump_valve.toml must parse");
        assert_eq!(s.meta.name, "tank_pump_valve");
        assert_eq!(s.nodes.len(), 4);
        assert_eq!(s.pipes.len(), 3);
        assert_eq!(s.fidelity.flow, "newton");
        // IndexMap preserves file order → deterministic node ids.
        assert_eq!(s.nodes.get_index(0).unwrap().0, "supply_tank");
    }

    /// `ambient_ua_w_per_k` is optional and defaults to a perfectly insulated
    /// tank. This is what keeps every scenario written before the field existed
    /// bit-identical — `tank_pump_valve.toml` does not mention it, and must
    /// still load a tank with UA = 0 rather than failing to parse or picking up
    /// some other number.
    #[test]
    fn a_tank_without_an_ambient_ua_is_perfectly_insulated() {
        use refinery_core::graph::NodeKind;

        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).unwrap();
        let engine = super::build_engine(&s).expect("reference plant must build");
        let id = engine.graph.find_node("supply_tank").unwrap();
        match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => assert_eq!(
                t.ambient_ua.value(),
                0.0,
                "a tank whose file omits ambient_ua_w_per_k must be insulated"
            ),
            _ => panic!("supply_tank must be a tank"),
        }
    }

    /// The pipe field is optional too, and it matters more than the tank's: a
    /// pipe is the one body EVERY scenario has, so a nonzero default would move
    /// the answer of every file ever written rather than only those with tanks.
    #[test]
    fn a_pipe_without_an_ambient_ua_is_perfectly_insulated() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).unwrap();
        let engine = super::build_engine(&s).expect("reference plant must build");
        for edge in engine.graph.edge_ids() {
            let pipe = engine.graph.pipe(edge);
            assert_eq!(
                pipe.ambient_ua.value(),
                0.0,
                "pipe '{}' omits ambient_ua_w_per_k and must be insulated",
                pipe.name
            );
        }
    }

    /// A pipe `UA` written in a file reaches the pipe unchanged, and is already
    /// SI for the same reason the tank's is.
    #[test]
    fn an_ambient_ua_reaches_the_pipe_in_watts_per_kelvin() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml").replace(
            "name = \"suction\"",
            "name = \"suction\"\nambient_ua_w_per_k = 750.0",
        );
        let s = super::load_str(&src).expect("must parse with an ambient_ua_w_per_k");
        let engine = super::build_engine(&s).expect("must build");
        let suction = engine
            .graph
            .edge_ids()
            .find(|e| engine.graph.pipe(*e).name == "suction")
            .expect("the suction pipe must exist");
        assert_eq!(
            engine.graph.pipe(suction).ambient_ua.value(),
            750.0,
            "750 W/K in the file must be 750 W/K on the pipe, unscaled"
        );
    }

    /// A negative pipe `UA` is refused at LOAD, not discovered at solve. In a
    /// pipe the sign lands in an exponent, so `UA < 0` turns the decay toward
    /// ambient into growth away from it — a plant that diverges rather than one
    /// that merely reports a wrong number.
    #[test]
    fn a_negative_pipe_ambient_ua_is_refused() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml").replace(
            "name = \"suction\"",
            "name = \"suction\"\nambient_ua_w_per_k = -1.0",
        );
        let file = super::load_str(&src).expect("must parse");
        match super::build_engine(&file) {
            Ok(_) => panic!("a negative pipe ambient_ua_w_per_k must not build"),
            Err(e) => {
                let message = e.to_string();
                assert!(
                    matches!(e, refinery_core::error::SimError::Scenario(_))
                        && message.contains("suction")
                        && message.contains("conductance"),
                    "the error must name the pipe and say why, got: {message}"
                );
            }
        }
    }

    /// A `UA` written in a file reaches the tank unchanged. Unlike every other
    /// quantity in the format it is ALREADY SI — there is no customary unit for
    /// it worth converting from — so what this pins is the absence of a
    /// conversion: bar→Pa and °C→K next door make a stray factor here the
    /// natural mistake.
    #[test]
    fn an_ambient_ua_reaches_the_tank_in_watts_per_kelvin() {
        use refinery_core::graph::NodeKind;

        let src = include_str!("../../../scenarios/tank_pump_valve.toml").replace(
            "[nodes.supply_tank]",
            "[nodes.supply_tank]\nambient_ua_w_per_k = 500.0",
        );
        let s = super::load_str(&src).expect("must parse with an ambient_ua_w_per_k");
        let engine = super::build_engine(&s).expect("must build");
        let id = engine.graph.find_node("supply_tank").unwrap();
        match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => assert_eq!(
                t.ambient_ua.value(),
                500.0,
                "500 W/K in the file must be 500 W/K in the tank, unscaled"
            ),
            _ => panic!("supply_tank must be a tank"),
        }
    }

    /// A negative `UA` is refused at load. It is a CONDUCTANCE, not a signed
    /// rate: direction already comes from `(T_ambient − T_tank)`, so a negative
    /// value does not mean "loses heat" — it inverts the driving force into
    /// positive feedback, warming a hot tank further. That plant RUNS and
    /// reports finite temperatures the whole way, which is why it has to be
    /// stopped at the file rather than caught downstream.
    #[test]
    fn a_negative_ambient_ua_is_refused() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let mut file = super::load_str(src).expect("must parse");
        match file.nodes.get_mut("supply_tank").expect("a supply_tank") {
            super::NodeDef::Tank {
                ambient_ua_w_per_k, ..
            } => *ambient_ua_w_per_k = -1.0,
            other => panic!("supply_tank must be a tank, got {other:?}"),
        }
        // `Engine` is not `Debug`, so unwrap the Result by hand.
        match super::build_engine(&file) {
            Ok(_) => panic!("a negative ambient_ua_w_per_k must not build"),
            Err(e) => {
                let message = e.to_string();
                assert!(
                    matches!(e, refinery_core::error::SimError::Scenario(_))
                        && message.contains("supply_tank")
                        && message.contains("conductance"),
                    "must fail as a Scenario error naming the tank and explaining \
                     why, got: {message}"
                );
            }
        }
    }

    /// The reference plant must build into a wired graph and actually run:
    /// this is the end-to-end proof that node/pipe instantiation, unit
    /// conversions, and the fold-at-source device direction are all correct.
    #[test]
    fn build_engine_wires_and_runs_the_reference_plant() {
        use refinery_core::graph::NodeKind;

        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).unwrap();
        let mut engine = super::build_engine(&s).expect("reference plant must build");
        assert_eq!(engine.graph.node_count(), 4);
        assert_eq!(engine.graph.edge_count(), 3);

        let receiving_mass = |e: &refinery_core::engine::Engine| {
            let id = e.graph.find_node("receiving_tank").unwrap();
            match &e.graph.node(id).kind {
                NodeKind::Tank(t) => t.mass.value(),
                _ => panic!("receiving_tank must be a tank"),
            }
        };
        let before = receiving_mass(&engine);

        // ~50 ticks exercises convergence + finiteness every tick (the 1000-tick
        // mass-balance run is a separate M1 acceptance gate). The pump drives
        // supply→receiving unambiguously (supply bottom ≈180 kPa, receiving
        // ≈111 kPa, pump adds up to ρg·40 ≈392 kPa against the 5 m ≈49 kPa
        // fill-line rise), so the receiving tank must gain mass.
        for _ in 0..50 {
            engine
                .tick()
                .expect("every tick must converge with no non-finite state");
        }
        let after = receiving_mass(&engine);
        assert!(
            after > before,
            "receiving tank must fill: before={before}, after={after}"
        );
    }

    #[test]
    fn unknown_solver_is_a_clear_error() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let mut s = super::load_str(src).unwrap();
        s.fidelity.flow = "quantum".into();
        let err = match super::build_engine(&s) {
            Err(e) => e,
            Ok(_) => panic!("expected an error for unknown solver"),
        };
        assert!(err.to_string().contains("quantum"));
    }
}
