//! The scenario file format: the TOML schema, its defaults, and `load_str`.
//!
//! Nothing here builds anything. A `ScenarioFile` is a parsed document, and
//! every decision about what it means is made in `build` and `validate`.

use refinery_core::error::SimError;
use refinery_core::units::{Kelvin, Pascal, T_AMBIENT};
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
    /// The plant's regulating loops (M8.2). Optional: absent is the thirteen
    /// scenarios written before M8, which regulate nothing and stay
    /// byte-identical because an empty list makes the loop pass and the snapshot
    /// field both vanish.
    #[serde(default)]
    pub controls: Vec<ControlDef>,
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
    /// "constant" (M1) | "trouton" (M7.3).
    ///
    /// Until M7.2 this string was parsed and then **ignored**: `build_engine`
    /// hardcoded `ConstantThermo`, so `thermo = "nonsense"` loaded a working
    /// plant. It was alone among the fidelity keys in that, and it was found by
    /// wiring `separation` beside it in M7.1 rather than by a test — nothing
    /// reached the value, so nothing could fail on it.
    ///
    /// **`"trouton"` was held back until M7.3, and the reason is worth keeping.**
    /// The only consumer of a K-value is the cascade, so before M7.3 selecting it
    /// would have changed no number in any plant — a scenario knob nothing can
    /// discriminate, which is precisely the shape M7.1 measured on `smearing_k`
    /// (`a-hand-written-scenario-can-be-vacuous`). It arrives together with the
    /// load-time refusal of the pairing it makes possible: `separation =
    /// "cascade"` with `thermo = "constant"` (see `build_engine`).
    ///
    /// The mirror pairing — `"trouton"` with the cut-point splitter — is still a
    /// knob that changes nothing, and is deliberately **not** refused: a K-value
    /// is a property of the fluid rather than of the column, and the next
    /// consumer of one need not be a separation model. That is the same
    /// permissiveness `reactions` already has toward a plant with no reactor.
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
    /// "none" (M1–M12.0) | "flash" (M12.1).
    ///
    /// What a liquid holdup does when it is above its own bubble point
    /// (docs/DESIGN.md §14). `"none"` says a product tank is a liquid store
    /// whose contents never boil however hot the column runs; `"flash"` says the
    /// superheat leaves as vapour at `y = K·x` through a vent to `Atmosphere`
    /// with nothing downstream of it. **Neither is a refinement of the other** —
    /// the first moves no mass and invents no stream, the second is incomplete
    /// in the way `docs/DEFERRED.md` B12 and B13 already record — which is what
    /// makes this a fidelity key rather than a bug fix with a flag on it (§14
    /// fork 9).
    ///
    /// Defaults to `"none"` on the `separation`/`phase` argument: that is what
    /// every file written before M12 MEANS, not merely what keeps it loading.
    /// It is forced anyway — fourteen of the sixteen shipped plants declare
    /// `thermo = "constant"`, which has no K-value to flash with.
    ///
    /// **The pairing that is refused, and the defect it is modelled on.**
    /// `boiloff = "flash"` with `thermo = "constant"` is a load-time error (see
    /// `require_compatible_fidelity`), exactly as `separation = "cascade"` with
    /// the same thermo is. And the value is READ on the day it lands: `thermo`
    /// was parsed and then ignored from M1 to M7.2, so `thermo = "nonsense"`
    /// loaded a working plant — a key whose value is consumed only when some
    /// other key is set is born in exactly that state, which is why
    /// `crude_column_boiloff.toml` ships as a one-line twin of
    /// `crude_column_cascade.toml` and the two must disagree.
    ///
    /// Its own default function rather than `reactions`' `default_none`,
    /// although both spell the same string: sharing one would make "flip the
    /// boil-off default" and "flip the reaction default" the same edit, and a
    /// key's default is exactly the sort of thing a mutation pass has to be able
    /// to move on its own.
    #[serde(default = "default_no_boiloff")]
    pub boiloff: String,
}

fn default_constant() -> String {
    "constant".into()
}

fn default_none() -> String {
    "none".into()
}

fn default_no_boiloff() -> String {
    "none".into()
}

fn default_cut_point() -> String {
    "cut_point".into()
}

fn default_total() -> String {
    "total".into()
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
        ///
        /// An `Option` rather than a plain default since M7.3, because the
        /// declared-iff-used correspondence needs to tell "absent" from "written
        /// as 0". This is a cut-point knob and the cascade ignores it, so writing
        /// it on a cascade column is refused rather than silently unread.
        #[serde(default)]
        smearing_k: Option<f64>,
        /// Draws in ascending boiling-point order, lightest first — which is also
        /// top-down for a cascade. Each names the product node it feeds; how it is
        /// located and sized depends on the separation fidelity. See `DrawDef`.
        draws: Vec<DrawDef>,
        /// Equilibrium-stage equipment. Required iff `[fidelity] separation =
        /// "cascade"`, refused otherwise. See `CascadeDef`.
        #[serde(default)]
        cascade: Option<CascadeDef>,
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
/// Under the CASCADE fidelity the same list is read differently: `up_to_c` is
/// refused and each draw declares the `stage` it leaves from and the `draw_ratio`
/// (a **mass** fraction of the feed) it takes. The last draw is the bottoms and
/// declares neither a stage top nor a ratio — it leaves the reboiler and gets
/// `1 − Σ others`.
#[derive(Debug, Deserialize)]
pub struct DrawDef {
    /// Name of the product node this draw feeds.
    pub outlet: String,
    /// Top of this draw's boiling band [°C, absolute]. Omitted on (and only on)
    /// the heaviest, last draw. **Cut-point fidelity only.**
    #[serde(default)]
    pub up_to_c: Option<f64>,
    /// Which equilibrium stage this draw leaves from: `0` is the total condenser
    /// (the distillate), `stages` is the reboiler (the bottoms), and everything
    /// between is a liquid side draw. **Cascade fidelity only.**
    #[serde(default)]
    pub stage: Option<u32>,
    /// This draw's **mass** flow as a fraction of the column feed. Set on every
    /// draw but the last. **Cascade fidelity only.**
    #[serde(default)]
    pub draw_ratio: Option<f64>,
    /// `"liquid"` (the default and the only supported value) or `"vapour"`.
    ///
    /// The key exists so that a **vapour side draw** is something a file can say
    /// and be refused for, rather than something the format cannot express. DESIGN
    /// §5's energy argument buys M7's whole scope boundary from one condition —
    /// with a total condenser and all-liquid draws, every kilogram vaporized
    /// inside the column condenses inside it, the latent flows cancel, and the
    /// external balance stays purely sensible against the workspace's one datum.
    /// A vapour draw carries latent heat across the unit boundary and breaks that.
    /// **Cascade fidelity only.**
    #[serde(default)]
    pub phase: Option<String>,
}

/// A cascade column's equipment (M7.3): `[nodes.<column>.cascade]`.
///
/// What is deliberately absent is any absolute flow. The column is specified by
/// ratios alone — `reflux_ratio` here, and one `draw_ratio` per draw — because a
/// prescribed product rate in kg/s re-runs the failure that killed M3.2's
/// prescribed-draw column (DESIGN §5, fork 3). The total through the column stays
/// hydraulically determined; only the split is composition-determined.
#[derive(Debug, Deserialize)]
pub struct CascadeDef {
    /// Number of equilibrium stages `N`, **counting the reboiler and excluding
    /// the total condenser** (DESIGN §5, correction 3).
    pub stages: u32,
    /// Which stage the feed enters, in `1..=stages`. The feed is taken as a
    /// **saturated liquid**; feed quality is deferred (correction 5).
    pub feed_stage: u32,
    /// Reflux ratio `R = L/D` — molar, internal, and one of the two real
    /// control-room handles this fidelity exposes.
    pub reflux_ratio: f64,
    /// `"total"` (the default and the only supported value) or `"partial"`.
    ///
    /// Present for the same reason as `DrawDef::phase`: a partial condenser
    /// (vapour distillate) is refused at load, and a refusal of something the
    /// format cannot express is not a refusal.
    #[serde(default = "default_total")]
    pub condenser: String,
}

fn default_true() -> bool {
    true
}

/// Ambient in the scenario file's display units (°C), so the default round-trips
/// through `c_to_k` to exactly `T_AMBIENT` rather than to a near-miss constant.
fn default_ambient_c() -> f64 {
    T_AMBIENT.value() - 273.15
}

/// One `[[controls]]` entry: a regulating loop (M8.2, docs/DESIGN.md §10).
///
/// **`deny_unknown_fields`, alone among the definitions in this file**, and that
/// asymmetry is deliberate. Everywhere else an unrecognised key is a typo in a
/// quantity that has a visible consequence — a mistyped `length_m` fails to
/// deserialize because the field is required. Here the tuning constants are
/// per-algorithm and therefore optional at the serde level, so `gain_per_metre`
/// would parse as an unknown key, leave `gain_per_m` absent, and the file would
/// be refused for the wrong reason or (once a default existed) accepted with an
/// invented gain. The keys that belong to the OTHER algorithm are refused by name
/// below; this catches the ones that belong to no algorithm at all.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlDef {
    /// Unique per plant. What the loop's faceplate is labelled with.
    pub name: String,
    /// Which node's state this loop watches, and which of its variables.
    pub measurement: MeasurementDef,
    /// Name of the node this loop writes. A `valve`; a `relief_valve` is refused
    /// with its own reason.
    pub actuator: String,
    /// `"p"` (M8.2) or `"pi"` (M8.3). Each has its own required tuning keys, and
    /// each refuses the other's — the two-directional refusal the separation
    /// fidelities established.
    pub algorithm: String,
    /// `"auto"` (the loop drives its actuator) or `"manual"` (a human does).
    pub mode: String,
    /// The target, in metres. **Required for `variable = "level"` and refused on
    /// any other variable**, exactly as `up_to_c` is the splitter's and `stage`
    /// the cascade's.
    ///
    /// The unit is in the KEY because a snapshot's setpoint cannot put it in a
    /// field name (docs/DESIGN.md §10 fork 4). **This doc used to predict that
    /// pressure control's key would be `setpoint_pa`, and M10 followed the FORMAT
    /// instead of the prediction** (docs/DESIGN.md §12 fork 3): every pressure a
    /// scenario declares is in bar, so `setpoint_pa` would have been the only one
    /// that is not. The key is `setpoint_bar`, below.
    ///
    /// The refusing direction became reachable with it. Until a second variable
    /// existed, "this key belongs to the other variable" was not a state the
    /// format could reach — `deny_unknown_fields` refuses a key belonging to no
    /// variable at all, which is a different message — so by the project's own
    /// rule (a refusal of something the format cannot express is not a refusal)
    /// that pair is new work in M10 rather than existing coverage.
    #[serde(default)]
    pub setpoint_m: Option<f64>,
    /// The target, in bar absolute. **Required for `variable = "pressure"` and
    /// refused on any other variable** (docs/DESIGN.md §12 fork 3).
    ///
    /// Bar rather than Pascals because that is what the rest of the format says:
    /// `pressure_bar` on source, sink, vessel and column, `set_pressure_bar` and
    /// `accumulation_bar` on the PSV — six pressure keys across four node kinds
    /// and no `_pa` anywhere. A `setpoint_pa` would be a number whose unit a
    /// reader has to infer from its neighbours' *dis*agreement, which is fork 4's
    /// own failure mode inverted.
    ///
    /// Converted to Pascals at the loader together with `gain_per_bar`, at one
    /// site. See that key.
    #[serde(default)]
    pub setpoint_bar: Option<f64>,
    /// Proportional gain, per metre of level error. **Level loops only.**
    ///
    /// **The unit is in the key, and the design note wrote this one bare.** That
    /// is corrected here rather than followed: a gain is `1/m` on a level loop and
    /// `1/bar` on a pressure loop, so a bare `gain` is a number whose unit depends
    /// on a sibling key — which is the exact failure fork 4 spent its own
    /// correction on for `setpoint_m`, applied to the other half of the same
    /// entry. No default, for the reason `x_T` has none.
    #[serde(default)]
    pub gain_per_m: Option<f64>,
    /// Proportional gain, per BAR of pressure error. **Pressure loops only.**
    ///
    /// **Per bar, not per Pascal, and this is the slice's named trap**
    /// (docs/DESIGN.md §12 fork 3). A gain's unit is its setpoint's reciprocal, so
    /// a `setpoint_bar` forces a `gain_per_bar` — but a controller's arithmetic is
    /// in SI, so the loader must multiply the setpoint by 1e5 and DIVIDE this by
    /// the same 1e5. Converting one and not the other is a factor of 100 000 that
    /// no type catches, because a gain is a bare `f64` all the way into
    /// `ProportionalController::new` and the loop stays perfectly stable, merely
    /// mistuned by five orders of magnitude. Both conversions are written as one
    /// pair at one site in `build_controls`, and a gate measures the loop's first
    /// output against a hand-computed `K·e` rather than trusting either.
    #[serde(default)]
    pub gain_per_bar: Option<f64>,
    /// Integral time [s] — the ISA reset time, the interval in which the integral
    /// term alone repeats the proportional term's contribution.
    ///
    /// **PI only.** Required with `algorithm = "pi"` and refused on `"p"`, in both
    /// directions. The refusing direction shipped in M8.2, one slice before the
    /// algorithm that reads this key existed, for `DrawDef::phase`'s reason: a
    /// refusal of something the format cannot express is not a refusal. The
    /// requiring direction became reachable here, the moment `"pi"` became a
    /// selectable algorithm, which is exactly the expiry M8.2's note named.
    #[serde(default)]
    pub integral_time_s: Option<f64>,
    /// The actuator position the loop starts from, dimensionless in `[0, 1]`.
    ///
    /// **PI only, and it is the loop's MEMORY rather than a bias** (docs/DESIGN.md
    /// §10 fork 5). The integral term is *derived* from it at load by the same
    /// back-calculation MANUAL→AUTO uses, so there is exactly one way a loop's
    /// memory can be initialised and no silent zero anywhere — and the scenario
    /// declares one number rather than two.
    ///
    /// **Refused on `algorithm = "p"` with its own reason**, which is a change of
    /// message rather than of behaviour: until this key existed, a file writing it
    /// was refused by `deny_unknown_fields` as an unknown key, and a proportional
    /// loop rejecting it as *unknown* would now be a lie — the key exists, it just
    /// names a memory that controller does not have. M8.2's correction 2 is why it
    /// is not a bias: a manual-reset term would make the P loop's steady-state
    /// offset a function of how well the bias was chosen, and that offset is the
    /// discriminating half of gate 3.
    ///
    /// Accepted on a loop declared `mode = "manual"`, deliberately: the memory is
    /// real from load and the very first `set_controller_mode` re-seeds it from
    /// the actuator anyway, so the number is consumed rather than decorative — and
    /// a loop that starts in MANUAL and is put in AUTO on tick 1 would otherwise
    /// have to declare a key it is then refused.
    #[serde(default)]
    pub initial_output: Option<f64>,
}

/// The `measurement = { node = "...", variable = "..." }` inline table.
///
/// A table rather than two flat keys, because the pair is one thing: a variable
/// with no node names nothing, and the loader has to resolve them together to
/// know whether the node can answer for that variable at all.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeasurementDef {
    pub node: String,
    /// `"level"` (M8.2, a `tank`) or `"pressure"` (M10, a `vessel`). Temperature
    /// and flow stay deferred per-variable — see `MeasuredVariable`. Which key
    /// carries the setpoint and which carries the gain both follow from this.
    pub variable: String,
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

/// The file's pressures are bar (absolute); the engine's are pascal.
pub(crate) fn bar_to_pa(bar: f64) -> Pascal {
    Pascal(bar * 1e5)
}

/// The file's temperatures are °C; the engine's are kelvin.
pub(crate) fn c_to_k(celsius: f64) -> Kelvin {
    Kelvin(celsius + 273.15)
}

/// Metric Kv [m³/h at ΔP = 1 bar, SG = 1] → element cv_si used by
/// `valve_flow`: Q[m³/s] = cv_si·opening·√(dP_Pa/ρ_rel). Starting from the
/// Kv definition Q[m³/h] = Kv·√(dP_bar/SG), convert h→s (÷3600) and bar→Pa
/// inside the root (÷√1e5): cv_si = Kv / (3600·√1e5).
pub(crate) fn kv_to_cv_si(kv: f64) -> f64 {
    kv / (3600.0 * 1e5_f64.sqrt())
}
