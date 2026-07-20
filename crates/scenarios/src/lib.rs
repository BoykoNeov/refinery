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

use refinery_core::components::{Composition, PseudoComponent, Slate};
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::error::SimError;
use refinery_core::graph::{
    HeatExchangerCoupling, Node, NodeId, NodeKind, Pipe, PlantGraph, TankState,
};
use refinery_core::stream::Stream;
use refinery_core::traits::{FlowSolver, ReactionModel, ThermoModel};
use refinery_core::units::{
    JPerKgK, Kelvin, Kg, KgPerM3, KgPerMol, Meter, Pascal, SquareMeter, Watt, WattPerKelvin, P_ATM,
    T_AMBIENT,
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
    pub density_kg_per_m3: f64,
    pub cp_j_per_kg_k: f64,
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
    /// "constant" (M1) — expands in M2+
    #[serde(default = "default_constant")]
    pub thermo: String,
    /// "none" (M1) — expands in M4
    #[serde(default = "default_none")]
    pub reactions: String,
}
fn default_constant() -> String {
    "constant".into()
}
fn default_none() -> String {
    "none".into()
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
    Pump {
        h0_m: f64,
        a: f64,
        #[serde(default = "default_true")]
        on: bool,
    },
    Valve {
        kv: f64,
        opening: f64,
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
        graph.add_pipe(
            from,
            to,
            Pipe {
                name: pipe.name.clone(),
                length: Meter(pipe.length_m),
                diameter: Meter(pipe.diameter_m),
                friction_factor: pipe.friction_factor,
                elevation_change: Meter(pipe.elevation_change_m),
                leak_area: SquareMeter::ZERO,
                ambient_ua: WattPerKelvin(pipe.ambient_ua_w_per_k),
                // The solver overwrites mass_flow each tick, and transport
                // overwrites the temperature. Seed representative T/P at
                // ambient / atmospheric.
                stream: Stream::stagnant(slate.len(), T_AMBIENT, P_ATM),
            },
        );
    }

    // Step 2b: thermally pair the exchanger sides, after every node exists so
    // both ends of a coupling can be resolved regardless of file order.
    build_couplings(&mut graph, &scenario.exchangers)?;

    // Step 3: validate topology at load — a clear error here beats solve-time
    // divergence for the same structural fault.
    validate_topology(&graph, &slate)?;

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
    let thermo: Box<dyn ThermoModel> = Box::new(refinery_solvers::ConstantThermo);
    let reactions: Box<dyn ReactionModel> = Box::new(refinery_solvers::NoReactions);

    let config = EngineConfig {
        dt: refinery_core::units::Seconds(scenario.simulation.dt),
    };
    Ok(Engine::new(graph, slate, config, flow, thermo, reactions))
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
            ("density_kg_per_m3", def.density_kg_per_m3),
            ("cp_j_per_kg_k", def.cp_j_per_kg_k),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "component '{}' has a non-positive or non-finite {field} \
                     ({value}); every component property must be > 0",
                    def.name
                )));
            }
        }
    }
    Slate::new(
        defs.iter()
            .map(|d| PseudoComponent {
                name: d.name.clone(),
                tb: c_to_k(d.tb_c),
                molar_mass: KgPerMol(d.molar_mass_kg_per_mol),
                density: KgPerM3(d.density_kg_per_m3),
                cp: JPerKgK(d.cp_j_per_kg_k),
            })
            .collect(),
    )
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
        NodeDef::Pump { h0_m, a, on } => NodeKind::Pump {
            h0: Meter(*h0_m),
            a: *a,
            on: *on,
        },
        NodeDef::Valve { kv, opening } => NodeKind::Valve {
            cv_max: kv_to_cv_si(*kv),
            opening: *opening,
        },
        NodeDef::Furnace { duty_mw } => NodeKind::Furnace {
            duty: Watt(*duty_mw * 1e6),
        },
        NodeDef::Cooler { duty_mw } => NodeKind::Cooler {
            duty: Watt(*duty_mw * 1e6),
        },
        NodeDef::HeatExchanger => NodeKind::HeatExchanger,
        NodeDef::Junction => NodeKind::Junction,
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
        let fixes = fixed_pressure_node(graph.node(id), slate);
        *has_fixer.entry(root).or_insert(false) |= fixes;
        representative.entry(root).or_insert(id);
    }
    for (root, fixed) in &has_fixer {
        if !*fixed {
            let member = representative[root];
            return Err(SimError::Scenario(format!(
                "network component containing '{}' has no pressure-fixing node \
                 (needs at least one source/sink/atmosphere/tank); its hydraulic \
                 problem is singular",
                graph.node(member).name
            )));
        }
    }
    Ok(())
}

/// True if the node pins a pressure (Source/Sink/Atmosphere/Tank). Delegates to
/// the solver's `fixed_pressure` so load-time and solve-time agree on what
/// counts as a boundary.
fn fixed_pressure_node(node: &Node, slate: &Slate) -> bool {
    refinery_solvers::network::fixed_pressure(node, slate).is_some()
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
