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
    /// The plant's trips (M22, docs/DESIGN.md §26). Optional: absent is every
    /// scenario written before M22, which protects nothing and stays
    /// byte-identical because an empty list makes the trip pass and the
    /// snapshot field both vanish.
    #[serde(default)]
    pub trips: Vec<TripDef>,
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
    /// THE constant heat capacity [J/(kg·K)], and it keeps that meaning under
    /// every fidelity.
    ///
    /// **A shaped component does not re-interpret this key**, and refusing to let
    /// it is §20 fork 3's named trap. Nineteen files declare it as the constant;
    /// under a shape it would silently become "the value at some temperature", and
    /// which temperature is written nowhere — precisely B17's failure (a key
    /// documented as insulation, used as a condenser). A shaped component declares
    /// its own anchor pair below instead, and the loader REFUSES to let a plant
    /// select the shaped model while reading this number.
    ///
    /// **Required under `heat_capacity = "constant"` and REFUSED under
    /// `"linear"`**, in both directions, which is `density_kg_per_m3`'s rule one
    /// field up: a gas has no stored density and a shaped cut has no stored
    /// capacity, and a declared constant that nothing reads is an
    /// authoritative-looking number with no effect. Under a shape the capacity at
    /// any temperature — the datum included — comes out of the anchor pair, so
    /// there is nothing left for this key to say.
    #[serde(default)]
    pub cp_j_per_kg_k: Option<f64>,
    /// The temperature this cut's shaped capacity is quoted at [°C at this
    /// boundary, K inside] (M16.2).
    ///
    /// The three `cp_*` shape keys below are all-or-nothing: declaring one means
    /// declaring all three, refused otherwise. A shape with a slope and no anchor
    /// is not a partial specification, it is an ambiguous one.
    #[serde(default)]
    pub cp_shape_anchor_c: Option<f64>,
    /// The capacity AT `cp_shape_anchor_c` [J/(kg·K)].
    #[serde(default)]
    pub cp_shape_at_anchor_j_per_kg_k: Option<f64>,
    /// `dcp/dT` [J/(kg·K²)]. Refused negative — see `components::CpShape`.
    #[serde(default)]
    pub cp_shape_slope_j_per_kg_k2: Option<f64>,
    /// `"liquid"` (default) or `"gas"`. Absent means liquid, which is what
    /// every slate written before M5.2 meant — the default that keeps those
    /// files bit-identical rather than merely still-loading.
    #[serde(default)]
    pub phase: Option<String>,
}

impl ComponentDef {
    /// Whether this component declares a `cp` shape at all — ANY of the three
    /// keys.
    ///
    /// Deliberately "any", not "all": the all-or-nothing check lives in
    /// `build_slate`, and reporting a half-declared shape as "declares none"
    /// would send an author to the wrong error. One of these predicates says
    /// *whether the author was reaching for a shape*, the other says *whether
    /// they finished*.
    #[must_use]
    pub fn declares_cp_shape(&self) -> bool {
        self.cp_shape_anchor_c.is_some()
            || self.cp_shape_at_anchor_j_per_kg_k.is_some()
            || self.cp_shape_slope_j_per_kg_k2.is_some()
    }
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
    /// "constant" (M1–M16.1) | "linear" (M16.2).
    ///
    /// What a heat capacity does with temperature (docs/DESIGN.md §20).
    /// `"constant"` says one number per cut is the whole story — which is what
    /// every file written before M16 MEANS, not merely what keeps it loading, and
    /// what nineteen of the twenty shipped plants say. `"linear"` says each cut
    /// declares `cp(T) = cp_a + s·(T − T_a)` and the engine integrates it.
    ///
    /// **A key of its own rather than a third arm on `thermo`, and that is
    /// argued** (§20 fork 4). `thermo` already selects between a model with no
    /// vapour–liquid equilibrium at all and a latent-heat correlation, and
    /// fourteen of nineteen plants select the former; putting a heat capacity
    /// there would make one key select two unrelated properties and force any
    /// plant wanting a shape to acquire a K-value model it has no use for. That is
    /// M14's argument for keeping a condenser off a new key, pointed the other
    /// way.
    ///
    /// **Both directions of every pairing are refused at load**
    /// (`require_compatible_fidelity`): a shape declared while `"constant"` is
    /// selected is a number nothing reads; `"linear"` with no `[[components]]`
    /// block, or with a component that declares no shape, is a model with no
    /// data; and `"linear"` beside `separation = "cascade"` or beside a valve
    /// declaring `x_t` is a plant HALF of whose energy arithmetic would still be
    /// reading the constant — the two sites this milestone deliberately did not
    /// reach.
    #[serde(default = "default_constant_heat_capacity")]
    pub heat_capacity: String,
}

fn default_constant_heat_capacity() -> String {
    "constant".into()
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
        /// External heat exchange with the surroundings: heat transfer
        /// coefficient × area, `UA` [W/K]. Already SI — there is no customary
        /// unit for it worth converting from, unlike the bar/°C/MW elsewhere in
        /// this file.
        ///
        /// **Named for the TERM, not for one use of it** (M15.1,
        /// docs/DESIGN.md §17 fork 4). It drives `Q = UA·(T_AMBIENT − T_tank)`,
        /// which is signed: heat leaking out of a hot tank through poor lagging
        /// and heat pulled out of a recovery drum by an ambient-cooled
        /// condenser are the same arithmetic with opposite intent. The old
        /// name, `ambient_ua_w_per_k`, read as the first of those and the only
        /// declaration in the shipped corpus is the second.
        ///
        /// Optional, defaulting to 0: a body that exchanges no heat with its
        /// surroundings — a perfectly insulated tank — which is what every
        /// scenario written before this field existed meant. See
        /// `TankState::ambient_ua`, whose name is deliberately NOT this one:
        /// `NodeSnapshot::kind` publishes the core struct, so renaming there
        /// would move every plant's bytes for a spelling.
        #[serde(default)]
        ambient_exchange_ua_w_per_k: f64,
        /// The retired spelling, kept only so that a file using it is REFUSED
        /// by name (`validate_node_def`) instead of silently losing its `UA`.
        ///
        /// `NodeDef` carries no `deny_unknown_fields` — see `ControlDef`, which
        /// is the one definition in this file that does — so without this
        /// tombstone an old file would parse, drop the key, and run a condenser
        /// with `UA = 0`: a plant that loads, ticks, and recovers a third of
        /// what its author asked for. M6.0's rule, in the direction nobody
        /// checks: a key nothing reads is a file that looks configured and is
        /// not.
        #[serde(default, rename = "ambient_ua_w_per_k")]
        retired_ambient_ua_w_per_k: Option<f64>,
        /// Where this tank's boil-off vent goes (M14, docs/DESIGN.md §16
        /// fork 2). The name of an `atmosphere` node or of another `tank`.
        ///
        /// Optional, and absent means the atmosphere `build_boiloff_vents`
        /// already finds or builds — which is what every file written before
        /// M14 means, so the whole existing corpus is untouched by construction
        /// rather than by measurement. Naming an atmosphere explicitly is legal
        /// and is that default written out.
        ///
        /// **On the TANK variant, not on `PipeDef`.** A vent is graph surgery
        /// performed at load (the `leak_to` precedent), so there is no pipe in
        /// the file to hang it on; and only a holdup can emit one, so putting
        /// the key here makes `vent_to` on a valve or a column unrepresentable
        /// instead of a runtime refusal. It is still refused on a plant whose
        /// `[fidelity] boiloff` builds no vents at all — see `build.rs`.
        #[serde(default)]
        vent_to: Option<String>,
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
    /// Check (non-return) valve (M30, docs/DESIGN.md §33). In gas service it
    /// takes `x_t` under `Valve`'s rule (M31, §34).
    CheckValve {
        kv: f64,
        /// Forward pressure difference [bar] across the valve at which the disc
        /// reaches full lift; it opens from zero. Required, no default: a
        /// datasheet number, and a silent default would be an invented one.
        /// Must be > 0, for the relief valve's `accumulation_bar` reason.
        full_open_bar: f64,
        /// As `Valve::x_t`: required in gas service, refused in liquid.
        x_t: Option<f64>,
    },
    /// Fired heater. Duty in MW — the unit refinery heaters are actually
    /// specified in, converted to W at this boundary like every other
    /// human-friendly quantity in the file.
    ///
    /// The three `coil_*` keys are its tube coil (M34, docs/DESIGN.md §37):
    /// the metal the duty heats, and through which it reaches the fluid. All
    /// three are REQUIRED, with no default: nothing in the engine derives a
    /// coil's mass or its film coefficient from a duty, and a default would be
    /// an invented number on every furnace that forgot it.
    Furnace {
        duty_mw: f64,
        /// Heat capacity of the tube metal [MJ/K] — its mass times steel's
        /// specific heat. Must be finite and > 0.
        coil_heat_capacity_mj_per_k: f64,
        /// Metal-to-process conductance `UA` [kW/K]: inside film coefficient
        /// times wetted area. Must be finite and > 0.
        coil_ua_kw_per_k: f64,
        /// The coil's temperature at load [°C]: a STATE, like a tank's
        /// `temperature_c`. Finite and above absolute zero.
        coil_temperature_c: f64,
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
    /// Which node's state — or, for a flow, which pipe's — this loop watches, and
    /// which variable.
    pub measurement: MeasurementDef,
    /// Name of the node this loop writes. A `valve` on a level, pressure or flow
    /// loop — on a flow loop, the valve whose own inlet or outlet pipe is measured
    /// (M20, docs/DESIGN.md §24 fork 3); a `cooler` (M17, §21 fork 3) or a
    /// `furnace` (M18, §22) on a temperature loop. Every other pairing is refused
    /// with its own reason, and a `relief_valve` always.
    ///
    /// **Or `{ loop = "<name>" }`** (M25, docs/DESIGN.md §29 fork 1): this loop is
    /// a cascade PRIMARY and its output is the named loop's setpoint, as a
    /// fraction of the `range_min_*`/`range_max_*` keys below. A bare string keeps
    /// meaning a node, so every file written before M25 parses unchanged.
    pub actuator: ActuatorDef,
    /// `"p"` (M8.2) or `"pi"` (M8.3). Each has its own required tuning keys, and
    /// each refuses the other's — the two-directional refusal the separation
    /// fidelities established.
    pub algorithm: String,
    /// `"auto"` (the loop drives its actuator) or `"manual"` (a human does).
    pub mode: String,
    /// `"direct"` or `"reverse"` — which way the loop's output moves its
    /// measurement (M18, docs/DESIGN.md §22). **Direct**: raising the output
    /// lowers the measurement (a drain, a vent, a cooler). **Reverse**: raising
    /// it raises the measurement (a furnace).
    ///
    /// **Absent means direct**, and that is a true statement rather than a
    /// guess: every loop written before M18 is direct, because reverse action was
    /// refused when it was written. The declaration is CHECKED against the
    /// actuator where the sign is physics, in both directions: a `cooler` must be
    /// direct and a `furnace` must say `"reverse"` — a furnace loop with the key
    /// absent is refused rather than defaulted, so a file's most surprising
    /// property is never invisible. **A flow loop must say `"reverse"` too**, by
    /// the same rule (M20, docs/DESIGN.md §24 fork 3): opening a valve raises the
    /// flow in its own pipe, and a valve has exactly one inlet and one outlet, so
    /// the loader checks that sign in one hop. **A level or pressure loop's valve
    /// is held to its side of the holdup** (M29, docs/DESIGN.md §32): a drain is
    /// direct (absent or `"direct"`), a fill must say `"reverse"`, and a valve
    /// that is neither, one pipe away, is refused either way.
    #[serde(default)]
    pub action: Option<String>,
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
    /// The target, in °C. **Required for `variable = "temperature"` and refused
    /// on any other variable** (M17, docs/DESIGN.md §21 fork 5).
    ///
    /// °C because every temperature the format declares is (`temperature_c`,
    /// `tb_c`, `up_to_c`, `t_set_c`), converted to kelvin by `+ 273.15` at the
    /// loader — an OFFSET, which is why `gain_per_k` beside it converts by nothing.
    #[serde(default)]
    pub setpoint_c: Option<f64>,
    /// Proportional gain, per KELVIN of temperature error. **Temperature loops
    /// only.**
    ///
    /// **Per K beside a setpoint in °C, and that is the slice's named trap,
    /// inverted from `gain_per_bar`'s** (docs/DESIGN.md §21 fork 5). A gain
    /// multiplies a temperature DIFFERENCE, and 1 °C of difference is 1 K, so
    /// this key is passed through untouched while the setpoint gains 273.15.
    /// Copying the pressure pair's "convert both at one site" would add the offset
    /// to the gain — 0.05 per K becoming 273.2 per K. The key says `_k` so its
    /// name carries the answer, as `smearing_k` does.
    #[serde(default)]
    pub gain_per_k: Option<f64>,
    /// The target, in kg/s, signed by the measured pipe's declared direction.
    /// **Required for `variable = "flow"` and refused on any other variable**
    /// (M20, docs/DESIGN.md §24 fork 4). Finite and strictly positive.
    ///
    /// **The first setpoint key whose file unit IS its SI unit**: the engine
    /// publishes kg/s (`EdgeSnapshot::stream.mass_flow`), so nothing converts
    /// here, and the conversion trap `gain_per_bar` and `gain_per_k` each guard
    /// against cannot be written. An operator's t/h or m³/h is a display
    /// conversion at the frontend (rule 4).
    #[serde(default)]
    pub setpoint_kg_per_s: Option<f64>,
    /// Proportional gain, per kg/s of flow error. **Flow loops only.** Converted
    /// by nothing, like its setpoint.
    #[serde(default)]
    pub gain_per_kg_per_s: Option<f64>,
    /// The cooler or furnace duty, in MW, that the loop's full output stands for
    /// — the loop's authority over a DUTY actuator. **Required when the actuator
    /// is a `cooler` or a `furnace` (M18) and refused when it is a `valve`**, in
    /// both directions (docs/DESIGN.md §21 fork 3), the `density_kg_per_m3` rule.
    ///
    /// On the loop rather than on the unit, because the unit's own fields are
    /// published on every snapshot and this number is the loop's statement of its
    /// range, read by nothing else. Must be finite and > 0; the unit's declared
    /// `duty_mw` must lie within `[0, max_duty_mw]` (§21 fork 4).
    #[serde(default)]
    pub max_duty_mw: Option<f64>,
    /// A cascade primary's range over a TEMPERATURE secondary, in °C: the
    /// secondary setpoints its output `0` and `1` stand for (M25, docs/DESIGN.md
    /// §29 fork 2). **Required on a primary whose secondary measures a
    /// temperature, refused everywhere else**, both directions — the
    /// `max_duty_mw` rule. Each end takes `+ 273.15`, as `setpoint_c` does; there
    /// is no span key, which would take nothing (the `setpoint_c`/`gain_per_k`
    /// trap, §21 fork 5). Both ends must pass the secondary's own setpoint check,
    /// with `min < max` strictly, and the secondary's declared setpoint must lie
    /// between them.
    #[serde(default)]
    pub range_min_c: Option<f64>,
    /// The top of `range_min_c`'s range, in °C.
    #[serde(default)]
    pub range_max_c: Option<f64>,
    /// A cascade primary's range over a FLOW secondary, in kg/s, converted by
    /// nothing (M25, §29 fork 2). As `range_min_c`, for a flow. **It cannot start
    /// at zero**: a flow setpoint of zero is refused ("shut the valve" is a MANUAL
    /// action, §24 fork 4), so a level loop over a drain's flow has a minimum
    /// flow — a minimum-flow stop, which is real practice.
    #[serde(default)]
    pub range_min_kg_per_s: Option<f64>,
    /// The top of `range_min_kg_per_s`'s range, in kg/s.
    #[serde(default)]
    pub range_max_kg_per_s: Option<f64>,
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

/// What a `[[controls]]` entry writes: a node by bare name, or another loop's
/// setpoint by `{ loop = "<name>" }` (M25, docs/DESIGN.md §29 fork 1).
///
/// **Untagged, so the bare string keeps its pre-M25 meaning**, and every file
/// written before cascade control parses unchanged. The price is serde's message
/// for a table that is neither shape — `{ loops = "…" }` is refused, but as "did
/// not match any variant of untagged enum ActuatorDef" rather than by the key's
/// name. Accepted: the refusal is still a refusal, and the table's own
/// `deny_unknown_fields` is what makes it one.
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum ActuatorDef {
    /// A valve, cooler or furnace, by node name.
    Node(String),
    /// A cascade secondary, by loop name.
    Loop(LoopActuatorDef),
}

/// The `{ loop = "<name>" }` inline table of [`ActuatorDef::Loop`].
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LoopActuatorDef {
    /// The secondary loop's `name`.
    #[serde(rename = "loop")]
    pub name: String,
}

impl std::fmt::Display for ActuatorDef {
    /// What a refusal calls the actuator: the node's name, or `loop '<name>'`.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ActuatorDef::Node(name) => f.write_str(name),
            ActuatorDef::Loop(def) => write!(f, "loop '{}'", def.name),
        }
    }
}

/// The `measurement = { node = "...", variable = "..." }` inline table, or
/// `measurement = { pipe = "...", variable = "flow" }` (M20).
///
/// A table rather than two flat keys, because the pair is one thing: a variable
/// with no point names nothing, and the loader has to resolve them together to
/// know whether the point can answer for that variable at all.
///
/// **Exactly one of `node` and `pipe`**, refused at load in both directions
/// (docs/DESIGN.md §24 fork 1). A flow asked of a `node`, and a level, pressure
/// or temperature asked of a `pipe`, are each refused with a message naming the
/// other key. `deny_unknown_fields` still refuses a key that is neither.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeasurementDef {
    /// The measured node, for a level, pressure or temperature.
    #[serde(default)]
    pub node: Option<String>,
    /// The measured pipe, for a flow (M20). Looked up among this file's
    /// DECLARED `[[pipes]]` only — the graph also holds edges the loader made (a
    /// leak split's `__downstream` half, every boil-off vent), and a file naming
    /// one would be metering a pipe it never declared.
    #[serde(default)]
    pub pipe: Option<String>,
    /// `"level"` (M8.2, a `tank`), `"pressure"` (M10, a `vessel`),
    /// `"temperature"` (M17, a `tank` or a `vessel`; M19, a furnace or cooler
    /// outlet) or `"flow"` (M20, one of the actuating valve's own two pipes).
    /// Which key carries the setpoint and which carries the gain both follow
    /// from this.
    pub variable: String,
}

/// One `[[trips]]` entry: a latching trip (M22, docs/DESIGN.md §26).
///
/// **`deny_unknown_fields`, for `ControlDef`'s reason**: every key but the name
/// is optional at the serde level so each can be refused with its own message,
/// and a misspelt `limit_metres` would otherwise parse, leave the real key
/// absent, and be refused for the wrong reason.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TripDef {
    /// Unique per plant. What the trip is labelled with in a snapshot.
    pub name: String,
    /// Which node's state or pipe's flow this trip watches, and which variable —
    /// the loops' own table. A tank's `level`, a vessel's `pressure` and a
    /// tank's or vessel's `temperature` exist from load; a declared pipe's
    /// `flow` (M33, docs/DESIGN.md §36) is absent on tick 1 alone, and the trip
    /// first compares on tick 2. A furnace or cooler outlet is refused by name
    /// (docs/DEFERRED.md E13).
    pub measurement: MeasurementDef,
    /// `"high"` (fires AT OR ABOVE the limit) or `"low"` (at or below).
    /// **Required, no default**: it is the trip's most important word.
    #[serde(default)]
    pub direction: Option<String>,
    /// The limit for a `level` trip, in metres, within `[0, height_m]` of the
    /// tank.
    #[serde(default)]
    pub limit_m: Option<f64>,
    /// The limit for a `pressure` trip, in bar absolute, converted to Pascals
    /// at the SAME site as a loop's `setpoint_bar` (docs/DESIGN.md §26 fork 6).
    #[serde(default)]
    pub limit_bar: Option<f64>,
    /// The limit for a `temperature` trip, in °C, given its `+ 273.15` at the
    /// same site as a loop's `setpoint_c`.
    #[serde(default)]
    pub limit_c: Option<f64>,
    /// The limit for a `flow` trip, in kg/s, signed by the pipe's declared
    /// direction (M33). Any finite value: a low trip at or below zero fires on
    /// a flow running backwards.
    #[serde(default)]
    pub limit_kg_per_s: Option<f64>,
    /// What the trip does when it fires: one or more pieces of equipment and
    /// each one's safe state. Required and non-empty (docs/DESIGN.md §26 fork 3).
    #[serde(default)]
    pub actions: Vec<TripActionDef>,
}

/// One entry of a trip's `actions` list: `{ pump = "…" }`,
/// `{ valve = "…", position = … }` or `{ furnace = "…" }`.
///
/// The equipment is named under the key for its KIND, so the file says what it
/// thinks it is pointing at and a `pump` action naming a valve is refused by
/// name rather than quietly doing a valve's job.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TripActionDef {
    /// A pump to stop.
    #[serde(default)]
    pub pump: Option<String>,
    /// A valve to put at `position`.
    #[serde(default)]
    pub valve: Option<String>,
    /// A furnace whose fuel to cut: its safe state is zero duty, so it takes no
    /// `position` (M32, docs/DESIGN.md §35).
    #[serde(default)]
    pub furnace: Option<String>,
    /// The valve's safe opening in `[0, 1]`. **Required on a valve, no default**:
    /// most trips close a valve, but a vent or dump valve trips OPEN, so the
    /// file says which. Refused on a pump, which has no position.
    #[serde(default)]
    pub position: Option<f64>,
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
    /// External heat exchange with the surroundings: heat transfer coefficient
    /// × exposed area, `UA` [W/K].
    ///
    /// Renamed with the tank's for one reason and kept spelled the same for
    /// another (M15.1, docs/DESIGN.md §17 fork 4): the two keys must agree,
    /// because a reader who learns one learns the other — and they do NOT drive
    /// the same equation. A tank's is a lumped `Q`; a pipe's is a transform
    /// along the edge. See `Pipe::ambient_ua`.
    ///
    /// **This key has never been declared in a shipped scenario** — measured
    /// across all eighteen files at M15.0 — so it is renamed on the strength of
    /// the tank's finding rather than on one of its own.
    ///
    /// Optional and defaulting to 0 — a perfectly insulated pipe — for the same
    /// reason as the tank's, and it matters more here: a pipe is the one body
    /// EVERY scenario has, so a nonzero default would change the answer of every
    /// file ever written rather than only those with tanks.
    #[serde(default)]
    pub ambient_exchange_ua_w_per_k: f64,
    /// The retired spelling. See the tank's — `PipeDef` carries no
    /// `deny_unknown_fields` either, so this is what makes an old file fail
    /// loudly rather than run un-lagged.
    #[serde(default, rename = "ambient_ua_w_per_k")]
    pub retired_ambient_ua_w_per_k: Option<f64>,
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
