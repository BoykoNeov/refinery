//! The load-time refusals: node and pipe definitions, fidelity pairings,
//! declared-iff-used config, and the built graph's topology.
//!
//! A fault caught here fails with a message naming the plant element; the
//! same fault caught later is a divergent solve with a residual history.

use refinery_core::components::{Composition, Phase, Slate};
use refinery_core::error::SimError;
use refinery_core::graph::{Node, NodeId, NodeKind, PlantGraph};
use refinery_core::traits::{LineFlashModel, ThermoModel};
use refinery_core::units::T_AMBIENT;
use std::collections::BTreeMap;

use crate::schema::{c_to_k, CascadeDef, DrawDef, NodeDef, PipeDef, ScenarioFile};

/// The retired `ambient_ua_w_per_k`, refused by name (M15.1, docs/DESIGN.md
/// §17 fork 4).
///
/// **A refusal rather than an alias, and rather than nothing.** Neither
/// `NodeDef` nor `PipeDef` carries `deny_unknown_fields`, so dropping the old
/// spelling from the schema would make an old file parse with the key ignored
/// and `UA = 0` — `crude_column_recovery.toml` would load, tick, and quietly
/// recover 20.7% instead of 42.4% with nothing anywhere saying why. Accepting
/// it as an alias instead would leave two spellings of one quantity in the
/// corpus forever, which is the failure `EdgeSnapshot::leak_mass_flow`'s own
/// comment records about two fields carrying one number.
fn retired_ambient_ua(element: &str, value: Option<f64>) -> Result<(), SimError> {
    match value {
        None => Ok(()),
        Some(v) => Err(SimError::Scenario(format!(
            "{element} declares ambient_ua_w_per_k = {v}, which was renamed to \
             ambient_exchange_ua_w_per_k in M15.1. The quantity is unchanged — it is the \
             same UA in the same W/K driving the same Q = UA·(T_AMBIENT − T_body) — and \
             only the name moved, because the term is external exchange with the \
             surroundings and the old name read as lagging alone (docs/DESIGN.md §17 \
             fork 4). Rename the key."
        ))),
    }
}

pub(crate) fn validate_pipe_def(def: &PipeDef) -> Result<(), SimError> {
    retired_ambient_ua(
        &format!("pipe '{}'", def.name),
        def.retired_ambient_ua_w_per_k,
    )?;
    if !def.ambient_exchange_ua_w_per_k.is_finite() || def.ambient_exchange_ua_w_per_k < 0.0 {
        return Err(SimError::Scenario(format!(
            "pipe '{}' has ambient_exchange_ua_w_per_k = {}: it must be finite and >= 0. UA \
             is a conductance; the DIRECTION of ambient exchange comes from \
             (T_ambient − T_in), so a negative value does not mean 'loses heat' — \
             in a pipe it inverts the exponent and drives the outlet away from \
             ambient without bound.",
            def.name, def.ambient_exchange_ua_w_per_k
        )));
    }
    Ok(())
}

pub(crate) fn validate_node_def(name: &str, def: &NodeDef) -> Result<(), SimError> {
    if let NodeDef::Tank {
        area_m2,
        height_m,
        initial_level_m,
        ambient_exchange_ua_w_per_k,
        retired_ambient_ua_w_per_k,
        ..
    } = def
    {
        // A tank's geometry, checked for the first time at M23 (docs/DESIGN.md
        // §27 fork 5). Once a tank has a brim, a shell with no footprint or no
        // height has no capacity, and every level divides by the area.
        for (key, value) in [("area_m2", area_m2), ("height_m", height_m)] {
            if !value.is_finite() || *value <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "tank '{name}' has {key} = {value}: a tank's footprint and height must \
                     be finite and > 0. A shell with no area or no height holds nothing, \
                     and its capacity ρ·A·H is what its overflow spills above \
                     (docs/DESIGN.md §27 fork 5)"
                )));
            }
        }
        if !initial_level_m.is_finite() || *initial_level_m < 0.0 {
            return Err(SimError::Scenario(format!(
                "tank '{name}' has initial_level_m = {initial_level_m}: a level must be \
                 finite and >= 0 (an empty tank is 0)"
            )));
        }
        // Admitted at equality: a tank declared exactly full ties exactly with
        // its capacity, because both are `TankState::mass_at_level` (fork 4).
        if initial_level_m > height_m {
            return Err(SimError::Scenario(format!(
                "tank '{name}' starts over its own brim: initial_level_m = \
                 {initial_level_m} m in a tank {height_m} m tall. It would spill the \
                 excess on its first tick with nothing saying why, so one of the two \
                 numbers is wrong (docs/DESIGN.md §27 fork 5)"
            )));
        }
        retired_ambient_ua(&format!("tank '{name}'"), *retired_ambient_ua_w_per_k)?;
        if !ambient_exchange_ua_w_per_k.is_finite() || *ambient_exchange_ua_w_per_k < 0.0 {
            return Err(SimError::Scenario(format!(
                "tank '{name}' has ambient_exchange_ua_w_per_k = \
                 {ambient_exchange_ua_w_per_k}: it must be finite and >= 0. UA is a \
                 conductance; the DIRECTION of ambient exchange comes from \
                 (T_ambient − T_tank), so a negative value does not mean 'loses heat' \
                 — it drives the tank away from ambient."
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
        if let Some(smearing_k) = smearing_k {
            if !smearing_k.is_finite() || *smearing_k < 0.0 {
                return Err(SimError::Scenario(format!(
                    "column '{name}' has smearing_k = {smearing_k}: it must be finite and >= 0 \
                     (a ramp width; 0 is a sharp splitter)."
                )));
            }
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
    if let NodeDef::Furnace {
        coil_heat_capacity_mj_per_k,
        coil_ua_kw_per_k,
        coil_temperature_c,
        flame_temperature_c,
        tube_failure_c,
        tube_rupture_area_cm2,
        fluid_heating_value_mj_per_kg,
        ..
    } = def
    {
        // The coil (M34, docs/DESIGN.md §37). Its step is exact, so no value is
        // refused for being small against `dt`; these are the physical bounds.
        if !coil_heat_capacity_mj_per_k.is_finite() || *coil_heat_capacity_mj_per_k <= 0.0 {
            return Err(SimError::Scenario(format!(
                "furnace '{name}' has coil_heat_capacity_mj_per_k = \
                 {coil_heat_capacity_mj_per_k}: it must be finite and > 0. A coil with no \
                 metal has no temperature of its own, which is the furnace this key replaced."
            )));
        }
        if !coil_ua_kw_per_k.is_finite() || *coil_ua_kw_per_k <= 0.0 {
            return Err(SimError::Scenario(format!(
                "furnace '{name}' has coil_ua_kw_per_k = {coil_ua_kw_per_k}: it must be finite \
                 and > 0. A coil that passes no heat to its fluid heats nothing."
            )));
        }
        if !coil_temperature_c.is_finite() || c_to_k(*coil_temperature_c).value() <= 0.0 {
            return Err(SimError::Scenario(format!(
                "furnace '{name}' has coil_temperature_c = {coil_temperature_c}: it must be \
                 finite and above absolute zero (−273.15 °C)."
            )));
        }
        // The flame (M36, docs/DESIGN.md §40): the flue law divides by
        // `T_f − T_a`, so a flame at or below the combustion air's temperature
        // would divide by zero or send the stack loss backwards.
        if !flame_temperature_c.is_finite() || c_to_k(*flame_temperature_c) <= T_AMBIENT {
            return Err(SimError::Scenario(format!(
                "furnace '{name}' has flame_temperature_c = {flame_temperature_c}: it must be \
                 finite and above the combustion air's {:.2} °C. A flame no hotter than the \
                 air it burns in heats nothing.",
                T_AMBIENT.value() - 273.15
            )));
        }
        // The tubes (M37, docs/DESIGN.md §42). A limit at or below the loaded
        // coil would burst the tubes on tick 1's pass, before the plant has run
        // at all; a limit at or above the flame is admitted and is a coil that
        // fuel alone can never burn out.
        if !tube_failure_c.is_finite() || *tube_failure_c <= *coil_temperature_c {
            return Err(SimError::Scenario(format!(
                "furnace '{name}' has tube_failure_c = {tube_failure_c}: it must be finite and \
                 above its coil_temperature_c = {coil_temperature_c}. Tubes loaded at or past \
                 their failure limit would burst before the plant has run."
            )));
        }
        if !tube_rupture_area_cm2.is_finite() || *tube_rupture_area_cm2 <= 0.0 {
            return Err(SimError::Scenario(format!(
                "furnace '{name}' has tube_rupture_area_cm2 = {tube_rupture_area_cm2}: it must \
                 be finite and > 0. A burn-out that opens no hole is not a burn-out."
            )));
        }
        if !fluid_heating_value_mj_per_kg.is_finite() || *fluid_heating_value_mj_per_kg < 0.0 {
            return Err(SimError::Scenario(format!(
                "furnace '{name}' has fluid_heating_value_mj_per_kg = \
                 {fluid_heating_value_mj_per_kg}: it must be finite and >= 0 (0 for a fluid \
                 that does not burn, such as water)."
            )));
        }
    }
    let duty = match def {
        NodeDef::Furnace { duty_mw, .. } => Some(("furnace", *duty_mw)),
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

/// The **declared-iff-used** correspondence on a column, in both directions.
///
/// `PseudoComponent::density` is required iff the component is liquid; a valve's
/// `x_T` is required iff it is in gas service. A column's separation config is the
/// same rule with two vocabularies rather than one: `up_to_c` and `smearing_k`
/// belong to the cut-point splitter, `stage`, `draw_ratio`, `phase` and the
/// `[cascade]` block belong to the cascade, and neither fidelity may carry the
/// other's fields (DESIGN §5, fork 2).
///
/// The point is not tidiness. A field a file declares and the running model never
/// reads is an authoritative-looking number that changes nothing — the shape M7.1
/// measured on `smearing_k` and recorded as
/// `a-hand-written-scenario-can-be-vacuous`. Refusing it at load is the only place
/// the mistake is still cheap.
pub(crate) fn require_declared_iff_used(
    name: &str,
    cascade_selected: bool,
    draws: &[DrawDef],
    cascade: &Option<CascadeDef>,
    smearing_k: &Option<f64>,
) -> Result<(), SimError> {
    if cascade_selected {
        let Some(def) = cascade else {
            return Err(SimError::Scenario(format!(
                "column '{name}' has no [nodes.{name}.cascade] block, but [fidelity] separation \
                 = \"cascade\" is selected: a stage cascade needs stages, feed_stage and \
                 reflux_ratio. A column with no equipment is the cut-point splitter's shape."
            )));
        };
        // The two scope boundaries DESIGN §5's energy argument buys the whole of
        // M7's fork 0 with. Both are refused HERE rather than in `core`, because
        // neither has any representation there — the refusal and the thing refused
        // would otherwise drift apart, which is what fork 0 asks not to happen.
        if def.condenser != "total" {
            return Err(SimError::Scenario(format!(
                "column '{name}' declares condenser = \"{}\": only \"total\" is supported. A \
                 PARTIAL condenser makes the distillate a vapour, which carries latent heat out \
                 of the unit — and M7's whole scope rests on the opposite: with a total \
                 condenser and all-liquid draws every kilogram vaporized inside the column \
                 condenses inside it, so the internal latent flows cancel and the external \
                 balance stays purely sensible against this workspace's one enthalpy datum. Two-\
                 phase transport ON THE GRAPH is the deferral this narrows to (DESIGN §5, fork 0).",
                def.condenser
            )));
        }
        if smearing_k.is_some() {
            return Err(SimError::Scenario(format!(
                "column '{name}' declares smearing_k under [fidelity] separation = \"cascade\": \
                 a smearing width is the cut-point splitter's ramp across a fixed boundary, and \
                 a cascade has no cut points to smear — its separation comes from stagewise \
                 equilibrium. The cascade would read nothing from it."
            )));
        }
        for d in draws {
            // Presence, not just exclusion. Without this the loader's
            // `stage.unwrap_or(0)` below would turn an omitted key into an
            // authoritative stage 0, `StageCascade::validate`'s "declares no
            // stage" arm would be unreachable from any FILE, and what the author
            // would see instead is a stage-ordering complaint pointing at the
            // wrong mistake. `draw_ratio` needs no equivalent — its `Option` is
            // passed straight through, so the model's own arm is reached.
            if d.stage.is_none() {
                return Err(SimError::Scenario(format!(
                    "column '{name}' draw '{}' declares no stage. Under [fidelity] separation = \
                     \"cascade\" every draw needs one: 0 is the total condenser (the \
                     distillate), {} is the reboiler (the bottoms), and 1..{} are liquid side \
                     draws.",
                    d.outlet, def.stages, def.stages
                )));
            }
            if d.up_to_c.is_some() {
                return Err(SimError::Scenario(format!(
                    "column '{name}' draw '{}' declares up_to_c under [fidelity] separation = \
                     \"cascade\": a cascade locates a draw by STAGE, not by a boiling-range top. \
                     Use stage = <n> (0 is the total condenser, {} the reboiler).",
                    d.outlet, def.stages
                )));
            }
            match d.phase.as_deref() {
                None | Some("liquid") => {}
                Some(other) => {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draw '{}' declares phase = \"{other}\": only \"liquid\" \
                         is supported. A VAPOUR side draw carries latent heat out of the unit, \
                         and M7's scope rests on every draw being liquid so the internal latent \
                         flows cancel and the external balance stays purely sensible. Two-phase \
                         transport on the graph is the deferral this narrows to (DESIGN §5, \
                         fork 0).",
                        d.outlet
                    )));
                }
            }
        }
    } else {
        if cascade.is_some() {
            return Err(SimError::Scenario(format!(
                "column '{name}' declares a [nodes.{name}.cascade] block, but [fidelity] \
                 separation is \"{}\": the cut-point splitter separates by boiling range and \
                 would read none of it. Select separation = \"cascade\" or delete the block.",
                "cut_point"
            )));
        }
        for d in draws {
            let stray = if d.stage.is_some() {
                Some("stage")
            } else if d.draw_ratio.is_some() {
                Some("draw_ratio")
            } else if d.phase.is_some() {
                Some("phase")
            } else {
                None
            };
            if let Some(key) = stray {
                return Err(SimError::Scenario(format!(
                    "column '{name}' draw '{}' declares {key}, which belongs to the cascade \
                     fidelity: the cut-point splitter locates a draw by its boiling-range top \
                     (up_to_c) and sizes it from the feed's own composition, so it would read \
                     nothing from {key}.",
                    d.outlet
                )));
            }
        }
    }
    Ok(())
}

/// Fidelity combinations that cannot work, refused before anything is built.
pub(crate) fn require_compatible_fidelity(scenario: &ScenarioFile) -> Result<(), SimError> {
    // **The pairing M7.2 left owing at this arm.** `ConstantThermo::k_value` is an
    // `Err`, so without this a mis-paired plant would load cleanly and fail on its
    // FIRST TICK — surfacing inside the composition sweep as a solver failure
    // rather than as the configuration mistake it is. A combination that cannot
    // work is a load-time error, exactly as an unknown model name is.
    if scenario.fidelity.separation == "cascade" && scenario.fidelity.thermo == "constant" {
        return Err(SimError::Scenario(
            "separation = \"cascade\" with thermo = \"constant\": a cascade stage IS a \
             vapour-liquid equilibrium, and the 'constant' fidelity is constant-property water \
             with no K-value to give it. Select thermo = \"trouton\"."
                .into(),
        ));
    }
    // The same shape, one key over, and refused for the same reason: without it
    // the plant loads happily, runs, and boils nothing at all — because
    // `ConstantThermo`'s `bubble_pressure` is an `Err` that the boil-off model
    // reads as "this fidelity cannot answer". That is silent rather than loud,
    // which makes it worse than the cascade's version: a scenario author would
    // see a plant that selects `flash` and behaves exactly like `none`.
    if scenario.fidelity.boiloff == "flash" && scenario.fidelity.thermo == "constant" {
        return Err(SimError::Scenario(
            "boiloff = \"flash\" with thermo = \"constant\": a boil-off IS a vapour-liquid              equilibrium — it needs a bubble point to park the holdup on, a K-value to say              what leaves, and a latent heat to size it — and the 'constant' fidelity is              constant-property water with none of the three. Select thermo = \"trouton\"."
                .into(),
        ));
    }
    require_compatible_heat_capacity(scenario)?;
    require_line_flash_plant(scenario)?;
    Ok(())
}

/// A plant selecting `[fidelity] line_flash = "equilibrium"` holds only what the
/// line flash models (M53, docs/DESIGN.md §58 fork 5) — refused at load, by
/// name, rather than run on a model that does not reach it.
///
/// - **A bubble pressure**: `thermo = "trouton"`. `"constant"` has no K-value,
///   so the plant would load, select the flash and boil nothing.
/// - **Liquids only**: a gas component is a different phase law
///   (`Composition::density_at`), not a stream that boils.
/// - **A tank boils off**: `boiloff = "flash"` on any plant with a tank. A tank
///   takes the latent heat of what arrives into its balance and its boil-off
///   vents it (fork 4); under `"none"` that heat would sit as superheat for ever.
/// - **Supplies, destinations, the atmosphere, junctions, control valves,
///   tanks and pumps.** A furnace, a cooler, an exchanger, a column, a reactor,
///   a vessel, and a check or relief valve are each a model not built, and a
///   boiling stream reaching one would be read as liquid. A pump in two-phase
///   service loses its head to RELAP5's multiplier (M54, §59).
/// - **No pump declares `npsh_required_m`** (M54, §59 decision 3): on a flashing
///   plant the steam-water data decides a pump's head, and it has a pump at its
///   boiling point with no vapour yet giving its whole head, where M50's curve
///   gives none. For the head not to jump at the bubble point the suction curve
///   would have to be reshaped until nothing of it is left, so the key would be
///   a number nothing reads.
/// - **No two pumps joined by one pipe** (M54, §59): each pump's inlet is
///   solved with its neighbours held, and two pumps back to back are one
///   problem, not two.
/// - **No leak path** (`leak_to`): a hole is an orifice, and a two-phase jet
///   chokes, which no law here models — the valve's caveat, without a user's
///   decision to accept it.
pub(crate) fn require_line_flash_plant(scenario: &ScenarioFile) -> Result<(), SimError> {
    if scenario.fidelity.line_flash != "equilibrium" {
        return Ok(());
    }
    let refuse = |why: String| {
        Err(SimError::Scenario(format!(
            "line_flash = \"equilibrium\" {why} (docs/DESIGN.md §58)"
        )))
    };
    if scenario.fidelity.thermo != "trouton" {
        return refuse(format!(
            "with thermo = \"{}\": a line flash IS a vapour-liquid equilibrium, and \
             only thermo = \"trouton\" has the bubble pressure and K-values it needs",
            scenario.fidelity.thermo
        ));
    }
    if let Some(gas) = scenario
        .components
        .iter()
        .find(|c| c.phase.as_deref() == Some("gas"))
    {
        return refuse(format!(
            "with a gas component ('{}'): the line flash boils liquids; a gas is carried \
             by its own phase law",
            gas.name
        ));
    }
    let has_tank = scenario
        .nodes
        .values()
        .any(|def| matches!(def, NodeDef::Tank { .. }));
    if has_tank && scenario.fidelity.boiloff != "flash" {
        return refuse(format!(
            "with boiloff = \"{}\" on a plant with a tank: a tank takes the latent heat \
             of the vapour that reaches it, and only boiloff = \"flash\" vents what that \
             heat boils",
            scenario.fidelity.boiloff
        ));
    }
    for (name, def) in &scenario.nodes {
        let kind = match def {
            NodeDef::Source { .. }
            | NodeDef::Sink { .. }
            | NodeDef::Atmosphere
            | NodeDef::Junction
            | NodeDef::Valve { .. }
            | NodeDef::Tank { .. }
            | NodeDef::Pump {
                npsh_required_m: None,
                ..
            } => continue,
            NodeDef::Pump { .. } => {
                return refuse(format!(
                    "with '{name}' declaring npsh_required_m: on a flashing plant a pump's \
                     head is RELAP5's two-phase multiplier on the vapour at its inlet \
                     (docs/DESIGN.md §59), which has a pump at its boiling point with no \
                     vapour giving its whole head. M50's suction curve gives it none, and \
                     cannot be reshaped to agree without leaving nothing of it: the number \
                     would change nothing. Remove the key"
                ));
            }
            NodeDef::Vessel { .. } => "a vessel",
            NodeDef::ReliefValve { .. } => "a relief valve",
            NodeDef::CheckValve { .. } => "a check valve",
            NodeDef::Furnace { .. } => "a furnace",
            NodeDef::Cooler { .. } => "a cooler",
            NodeDef::HeatExchanger => "a heat exchanger",
            NodeDef::Column { .. } => "a column",
            NodeDef::Reactor { .. } => "a reactor",
        };
        return refuse(format!(
            "with '{name}', {kind}: a plant that selects the line flash may hold only \
             supplies, destinations, the atmosphere, junctions, control valves, tanks \
             and pumps"
        ));
    }
    if let Some(pipe) = scenario.pipes.iter().find(|p| p.leak_to.is_some()) {
        return refuse(format!(
            "with a leak path on pipe '{}': a two-phase jet through a hole chokes, which \
             no law here models",
            pipe.name
        ));
    }
    // Each pump's inlet is solved with its neighbours held (M54, §59), so two
    // pumps joined by one pipe would each hold the other still: their two
    // balances are one problem, which no solve here poses.
    let is_pump = |name: &str| matches!(scenario.nodes.get(name), Some(NodeDef::Pump { .. }));
    if let Some(pipe) = scenario
        .pipes
        .iter()
        .find(|p| is_pump(&p.from) && is_pump(&p.to))
    {
        return refuse(format!(
            "with pipe '{}' joining two pumps: each pump's inlet is solved with its \
             neighbours held, and two pumps back to back are one problem, not two",
            pipe.name
        ));
    }
    Ok(())
}

/// The heat-capacity key against everything it cannot be paired with (M16.2,
/// docs/DESIGN.md §20 forks 4 and 5).
///
/// **Five refusals, and the last two are this milestone's own boundary rather
/// than a property of the physics.** `cascade.rs` computes a column's two duties
/// and its condenser's sensible term from `mixture_cp`, and `network.rs` computes
/// a gas valve's `γ = cp/cv` the same way; neither is reached through
/// `EnthalpyModel`, because both sit inside a solver trait whose signature this
/// slice deliberately did not change. A plant pairing them with a shape would
/// integrate its holdups on `cp(T)` and size its column duties and its choked
/// flow on a constant — a split this project refuses to ship silently. Refusing
/// makes the gap LOUD, which is the same move `boiloff = "flash"` with
/// `thermo = "constant"` made for a gap that was merely quiet.
fn require_compatible_heat_capacity(scenario: &ScenarioFile) -> Result<(), SimError> {
    let shaped = match scenario.fidelity.heat_capacity.as_str() {
        "constant" => false,
        "linear" => true,
        other => {
            return Err(SimError::Scenario(format!(
                "unknown heat capacity model '{other}' (valid: constant, linear)"
            )))
        }
    };

    // (1) A declared shape that nothing reads. The mirror of the gas-density
    //     refusal one field over: an authoritative-looking number with no effect.
    if !shaped {
        if let Some(def) = scenario.components.iter().find(|c| c.declares_cp_shape()) {
            return Err(SimError::Scenario(format!(
                "component '{}' declares a cp shape while [fidelity] heat_capacity = \"constant\", \
                 which reads only cp_j_per_kg_k: the shape is a number nothing reads. Select \
                 heat_capacity = \"linear\", or remove the cp_shape_* keys.",
                def.name
            )));
        }
    }

    // (1b) The mirror: under the constant model the key is the only source of a
    //      capacity, so its absence is not a default but a missing property.
    //      Checked here rather than by serde so the message can name the pairing.
    if !shaped {
        if let Some(def) = scenario
            .components
            .iter()
            .find(|c| c.cp_j_per_kg_k.is_none())
        {
            return Err(SimError::Scenario(format!(
                "component '{}' declares no cp_j_per_kg_k while [fidelity] heat_capacity = \
                 \"constant\", which has no other source for a heat capacity",
                def.name
            )));
        }
        return Ok(());
    }

    // (1c) And under a shape the same key has nothing left to say: the anchor
    //      pair gives the capacity at every temperature, this one included.
    if let Some(def) = scenario
        .components
        .iter()
        .find(|c| c.cp_j_per_kg_k.is_some())
    {
        return Err(SimError::Scenario(format!(
            "component '{}' declares cp_j_per_kg_k while [fidelity] heat_capacity = \"linear\", \
             which reads the cp_shape_* anchor pair instead: the constant is a number nothing \
             reads. Remove it, or select heat_capacity = \"constant\".",
            def.name
        )));
    }

    // (2) A model with no data at all — fork 5's six plants, which fall back to
    //     `Slate::water_only()` and a `cp` hard-coded in `core` that the loader
    //     never sees. They stay identical by construction because they cannot
    //     select this at all.
    if scenario.components.is_empty() {
        return Err(SimError::Scenario(
            "[fidelity] heat_capacity = \"linear\" on a plant with no [[components]] block: the \
             shape is declared per component, and this plant runs the water-only slate whose cp \
             is hard-coded in `core` and never passes through the loader (docs/DESIGN.md §20 \
             fork 5). Declare a [[components]] table, or select heat_capacity = \"constant\"."
                .into(),
        ));
    }

    // (3) A model with data for only some of its components. Partial is worse
    //     than absent: the mixture rule would silently drop a cut's contribution.
    if let Some(def) = scenario.components.iter().find(|c| !c.declares_cp_shape()) {
        return Err(SimError::Scenario(format!(
            "component '{}' declares no cp shape while [fidelity] heat_capacity = \"linear\": a \
             mixture's shape is the mass-weighted sum of its components', so one cut without one \
             leaves the model with no answer for any mixture containing it. Declare \
             cp_shape_anchor_c, cp_shape_at_anchor_j_per_kg_k and cp_shape_slope_j_per_kg_k2 on \
             every component, or select heat_capacity = \"constant\".",
            def.name
        )));
    }

    // (4) The cascade's duties and its condenser's sensible term.
    if scenario.fidelity.separation == "cascade" {
        return Err(SimError::Scenario(
            "heat_capacity = \"linear\" with separation = \"cascade\": a cascade column computes \
             its reboiler and condenser duties, and its condenser's sensible term, from the \
             CONSTANT mixture capacity — `SeparationModel::separate` is not handed the enthalpy \
             model, and M16.2 deliberately did not change that signature. The plant would \
             integrate its holdups on cp(T) and report duties on a constant. Deferred with a \
             trigger in docs/DEFERRED.md; select separation = \"cut_point\" or heat_capacity = \
             \"constant\"."
                .into(),
        ));
    }

    // (5) A gas valve's γ, computed inside the flow solve.
    if let Some(name) = scenario.nodes.iter().find_map(|(name, def)| match def {
        NodeDef::Valve { x_t: Some(_), .. }
        | NodeDef::ReliefValve { x_t: Some(_), .. }
        | NodeDef::CheckValve { x_t: Some(_), .. } => Some(name.clone()),
        _ => None,
    }) {
        return Err(SimError::Scenario(format!(
            "heat_capacity = \"linear\" with valve '{name}' declaring x_t: the compressible valve \
             law takes γ = cp/cv from the CONSTANT mixture capacity inside the flow solve, which \
             `FlowSolver::solve` reaches without the enthalpy model. The plant would choke on a \
             γ its own holdups no longer use. Deferred with a trigger in docs/DEFERRED.md; remove \
             x_t, or select heat_capacity = \"constant\"."
        )));
    }

    Ok(())
}

/// Validate the built graph at load time so a structural fault fails here with
/// a clear message rather than as a divergent solve later.
pub(crate) fn validate_topology(graph: &PlantGraph, slate: &Slate) -> Result<(), SimError> {
    // (a) Pump/valve (1-inlet, 1-outlet) degree. Reuse the solver's canonical
    //     invariant — single owner, no drift — remapped to a load-time error.
    refinery_solvers::network::validate_degrees(graph)
        .map_err(|e| SimError::Scenario(e.to_string()))?;

    // (b) Every connected component must contain at least one pressure-fixing
    //     node (Source/Sink/Atmosphere/Tank), else its hydraulic problem is
    //     singular. A tank is pinned AT LOAD, which is what this check asks; one
    //     that later runs dry is starved inside the solve and floats if nothing
    //     else anchors it (docs/DESIGN.md §28 fork 2, gate 10). Connectivity walks ALL pipes — NOT the solver's
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

/// A relief valve's `blowdown` needs a gas cushion at its inlet (M48,
/// docs/DESIGN.md §53 forks 4 and 4b): refused in liquid service, and refused in
/// gas unless a `Vessel` is reachable from its inlet through pipes and
/// zero-volume nodes, never through the relief itself.
///
/// The reason is the same one twice. The latch moves once per tick, and a pop
/// valve at full lift with nothing to store pressure behind it drops its own
/// inlet below reseat in the same solve, then stands back above set the tick
/// after it shuts: it flips every tick. A liquid line never has the cushion (its
/// holdups are vented tanks, a pump's discharge is algebraic), and liquid-trim
/// relief valves are not pop valves anyway. In gas, the user's DECISION
/// (2026-10-06) was to refuse a pop valve with no vessel behind it rather than
/// let it chatter on screen.
///
/// What it does NOT promise: a vessel behind an inlet line that loses more than
/// the blowdown at full lift still flips every tick — the real failure API 520
/// Part II's inlet-loss limit exists for, and a gate rather than a refusal. Nor
/// does it ask whether the way to the vessel can be SHUT: the search passes
/// through every zero-volume node, an operator's valve included, so a vessel
/// reached only through a valve someone has closed counts as a cushion it is
/// not.
///
/// Runs after `require_gas_valve_x_t`, and takes the service verdict from the
/// same `plant_phases` analysis.
pub(crate) fn require_blowdown_cushion(
    graph: &PlantGraph,
    phases: &[Phase],
) -> Result<(), SimError> {
    for nid in graph.node_ids() {
        let node = graph.node(nid);
        let NodeKind::ReliefValve {
            blowdown: Some(_), ..
        } = &node.kind
        else {
            continue;
        };
        if phases[nid.0 as usize] == Phase::Liquid {
            return Err(SimError::Scenario(format!(
                "relief valve '{}' declares blowdown_bar in liquid service. A blowdown                  valve pops to full lift and holds it until its inlet falls to reseat,                  which needs a gas cushion behind it; a liquid line has none, so it would                  flip open and shut every tick, and liquid-trim relief valves do not pop.                  Remove blowdown_bar (docs/DESIGN.md §53).",
                node.name
            )));
        }
        let mut seen = std::collections::BTreeSet::from([nid]);
        let mut frontier: Vec<NodeId> = graph
            .incident(nid)
            .into_iter()
            .filter(|(_, _, incoming)| *incoming)
            .map(|(_, other, _)| other)
            .collect();
        let mut cushioned = false;
        while let Some(at) = frontier.pop() {
            if !seen.insert(at) {
                continue;
            }
            match &graph.node(at).kind {
                NodeKind::Vessel(_) => {
                    cushioned = true;
                    break;
                }
                kind if refinery_core::energy::is_zero_volume(kind) => {
                    frontier.extend(graph.incident(at).into_iter().map(|(_, other, _)| other))
                }
                _ => {}
            }
        }
        if !cushioned {
            return Err(SimError::Scenario(format!(
                "relief valve '{}' declares blowdown_bar with no gas vessel behind its                  inlet. A blowdown valve pops to full lift and holds it until its inlet                  falls to reseat; with nothing to store pressure behind it, full lift drops                  the inlet below reseat at once and the valve flips open and shut every                  tick. Put a vessel on its inlet side, or remove blowdown_bar                  (docs/DESIGN.md §53).",
                node.name
            )));
        }
    }
    Ok(())
}

/// A pump's `npsh_required_m` needs a liquid to lose head in and a thermo model
/// that can say where that liquid boils (M50, docs/DESIGN.md §55). Refused in
/// gas service — a vapour does not cavitate, and the key would be a number
/// nothing reads — and on a plant whose `ThermoModel` has no bubble pressure,
/// which the model itself is asked rather than its name checked: it answers
/// `SimError::Scenario`, the refusal §13 already gives such a model. Without the
/// refusal the pump would silently deliver its whole curve forever, which is
/// the very disagreement the key exists to end.
///
/// Runs once the thermo model exists, and takes the service verdict from
/// `plant_phases`, as `require_blowdown_cushion` does.
pub(crate) fn require_pump_suction_answerable(
    graph: &PlantGraph,
    phases: &[Phase],
    thermo: &dyn ThermoModel,
    slate: &Slate,
) -> Result<(), SimError> {
    for nid in graph.node_ids() {
        let node = graph.node(nid);
        let NodeKind::Pump {
            suction: Some(_), ..
        } = &node.kind
        else {
            continue;
        };
        if phases[nid.0 as usize] != Phase::Liquid {
            return Err(SimError::Scenario(format!(
                "pump '{}' declares npsh_required_m in gas service. A vapour cannot                  cavitate — it is already vapour — so the key would be a number nothing                  reads. Remove npsh_required_m (docs/DESIGN.md §55).",
                node.name
            )));
        }
        let liquid = (0..slate.len())
            .map(|i| Composition::pure(slate.len(), i))
            .find(|c| matches!(c.phase(slate), Ok(Phase::Liquid)));
        let Some(liquid) = liquid else {
            continue;
        };
        if let Err(SimError::Scenario(why)) = thermo.bubble_pressure(slate, &liquid, T_AMBIENT) {
            return Err(SimError::Scenario(format!(
                "pump '{}' declares npsh_required_m, but this plant's thermo model has no                  bubble pressure to measure its suction against ({why}). Select                  thermo = \"trouton\" in [fidelity], or remove npsh_required_m                  (docs/DESIGN.md §55).",
                node.name
            )));
        }
    }
    Ok(())
}

/// Refuse a supply whose liquid boils at its own declared pressure (M52,
/// docs/DESIGN.md §57; ledger row B46, the user's DECISION: refused "for today,
/// modeling part-vapour supply in the future").
///
/// The engine carries a supply as one phase, so a supply above its bubble point
/// would feed liquid that is partly vapour as plain liquid. The rule is
/// `refinery_core::engine::supply_boiling`, the one the supply commands refuse
/// by, so a file and a command cannot disagree. Not asked of a gas supply, nor
/// on a plant whose thermo model has no bubble pressure (it cannot tell; the
/// snapshot says so), nor of a destination: its fluid matters only to a
/// back-feed, and M50's shipped destination stands below its own bubble
/// pressure.
pub(crate) fn require_supplies_below_boiling(
    graph: &PlantGraph,
    thermo: &dyn ThermoModel,
    line_flash: &dyn LineFlashModel,
    slate: &Slate,
) -> Result<(), SimError> {
    // A plant whose line flash carries vapour feeds a boiling supply as what it
    // is (M53, docs/DESIGN.md §58 fork 1): the refusal is lifted there and only
    // there.
    if line_flash.carries_vapour() {
        return Ok(());
    }
    for nid in graph.node_ids() {
        let node = graph.node(nid);
        let NodeKind::Source {
            pressure,
            temperature,
            composition,
        } = &node.kind
        else {
            continue;
        };
        if let refinery_core::snapshot::SupplyBoiling::Measured { bubble_pressure_pa } =
            refinery_core::engine::supply_boiling(thermo, slate, *temperature, composition)?
        {
            if bubble_pressure_pa > pressure.value() {
                return Err(SimError::Scenario(format!(
                    "source '{}' is boiling: at {:.2} °C its liquid boils at any pressure \
                     below {:.4} bar, and the source is declared at {:.4} bar. The engine \
                     carries a supply as liquid only on this plant, so it cannot feed one \
                     that is partly vapour (docs/DEFERRED.md B46). Raise pressure_bar or \
                     lower temperature_c (docs/DESIGN.md §57), or select [fidelity] \
                     line_flash = \"equilibrium\", which carries it (docs/DESIGN.md §58).",
                    node.name,
                    temperature.value() - 273.15,
                    bubble_pressure_pa / 1e5,
                    pressure.value() / 1e5
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
pub(crate) fn require_gas_valve_x_t(graph: &PlantGraph, phases: &[Phase]) -> Result<(), SimError> {
    for nid in graph.node_ids() {
        let node = graph.node(nid);
        // All three valve kinds, for one reason: they reach the same compressible
        // law through `compile_edge` (`fold_gas_service`), so they must share the
        // same requirement or one of them could reach the gas branch with no
        // `x_T`. The check valve joined at M31 (docs/DESIGN.md §34).
        let x_t = match &node.kind {
            NodeKind::Valve { x_t, .. }
            | NodeKind::ReliefValve { x_t, .. }
            | NodeKind::CheckValve { x_t, .. } => x_t,
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
pub(crate) fn plant_phases(graph: &PlantGraph, slate: &Slate) -> Result<Vec<Phase>, SimError> {
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
pub(crate) fn seed_component_index(slate: &Slate, phase: Phase) -> usize {
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
///   A tank stops being pinned while it is STARVED (M24, docs/DESIGN.md §28), but
///   that is a state reached during a run; at load every tank holds what it was
///   declared with, which is the question asked here.
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
