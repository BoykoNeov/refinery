//! Transport of the intensive fields: upwind (donor-cell) advection of
//! temperature and composition.
//!
//! Both ride ONE sweep (`resolve_node_states`). They share the topology — the
//! same inflow edges, the same upwind rule, the same ordering, the same recycle
//! rejection — and differ only in the weight each mix uses: enthalpy by `ṁ·cp`,
//! composition by `ṁ` alone. See `mix_compositions` for why that difference is
//! the sharpest edge in this module.
//!
//! The temperature field over the plant graph is *mixed*, and naming the two
//! kinds of node is what makes it tractable (docs/DESIGN.md §4a):
//!
//! - **Inertial nodes** (Tank, Source, Sink, Atmosphere) carry their own
//!   temperature. Tanks integrate it as a slow state; the reservoirs hold it
//!   fixed. Within a tick their temperature is a *boundary condition*, read at
//!   its start-of-tick value.
//! - **Zero-volume nodes** (Junction, Pump, Valve) hold no inventory, so their
//!   temperature is not a state at all — it is *algebraic*, the instantaneous
//!   enthalpy-weighted mix of whatever flows in.
//!
//! An edge's stream temperature is its **upwind** node's temperature, chosen by
//! the sign of the solved flow (so reverse flow is handled without a special
//! case). Zero-volume nodes therefore have to be resolved in flow order —
//! upstream before downstream — which is a topological sort over the *flow*
//! direction of this tick, not the graph's edge direction.
//!
//! That ordering exists unless a recycle passes through zero-volume nodes
//! **only**: any tank or reservoir in a loop breaks the dependency, because its
//! temperature is a start-of-tick constant rather than a function of its
//! inflows. A zero-volume-only recycle would need a simultaneous solve; it is
//! rejected as `SimError::Numerical` rather than silently mis-ordered. No
//! scenario in the workspace builds one (see `docs/DESIGN.md` §4a).
//!
//! Specific enthalpy is `h(T) = ∫_{T_REF}^{T} cp(τ) dτ` **for a LIQUID**, and the
//! qualifier is load-bearing rather than pedantic (M13, docs/DESIGN.md §15):
//! the reference state is *saturated liquid* at `T_REF`, so a VAPOUR on the
//! same datum is that plus its heat of vaporisation, `h = cp·(T − T_REF) + λ`.
//! Correct without the qualifier for every stream this engine carried before
//! M12.1 and wrong for the one it added — the same class of error as M5.3's
//! `u = cv·T − cp·T_REF`, which was right while a holdup's mass was constant
//! and load-bearing exactly when it was not. `Stream::latent` carries `λ` and
//! `EnthalpyModel::stream_enthalpy_flux` is the one expression that puts the two
//! together.
//!
//! **Since M16.2 the integral itself is a fidelity seam** (docs/DESIGN.md §20).
//! `h` is not `cp·(T − T_REF)` in this module any more: it is whatever
//! `traits::EnthalpyModel` says, and this module asks. `ConstantEnthalpy` gives
//! back the pre-M16 expression bit for bit; `LinearCpEnthalpy` integrates a
//! declared `cp(T)`. Every enthalpy flux, stock, mix and inversion in the engine
//! goes through that ONE model, so the reference cancels exactly as long as mass
//! balances — the invariant tests state it against `T_REF` explicitly rather than
//! assuming a zero reference makes it moot.

use crate::components::Phase;
use crate::components::{Composition, Slate};
use crate::error::SimError;
use crate::graph::{
    BedWall, CatalystBed, ColumnDraw, EdgeId, FurnaceCoil, FurnaceTubes, HydrogenConsumption,
    HydrogenKinetics, NodeId, NodeKind, PlantGraph, SteamInlet,
};
use crate::traits::{
    ColumnPass, ColumnSteam, DrawSeparation, EnthalpyModel, InflowEnthalpy, LineFlashModel,
    Reaction, ReactionModel, Separation, SeparationModel, Settled, StarvedTank, ThermoModel,
    VapourShare,
};
use crate::units::{
    JPerKgK, Kelvin, Kg, KgPerSec, Pascal, Seconds, Watt, WattPerKelvin, T_AMBIENT,
};
use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

/// Reference temperature for specific enthalpy [K]: the datum is **saturated
/// liquid at `T_REF`**, so `h = cp·(T − T_REF)` on a liquid and
/// `h = cp·(T − T_REF) + λ` on a vapour (`Stream::latent`, M13).
///
/// Deliberately non-zero (0 °C, the conventional steam-table datum). A zero
/// reference would make `h = cp·T` and hide any code path that forgot the
/// datum entirely; with 273.15 K, dropping it changes the answer, so the
/// energy tests actually discriminate on it.
pub const T_REF: Kelvin = Kelvin(273.15);

/// A holdup's end-of-tick composition: what it retained after the tick's outflow,
/// blended with what arrived.
///
/// ```text
/// m_c_new = f_c_old·(m_old − ṁ_out·dt) + ṁ_c,in·dt
/// ```
///
/// The outflow term is what makes this close. Fluid LEAVES at the holdup's
/// START-of-tick composition — that is what the upwind rule put on the outflow
/// edge, and what the far end was credited with — so the inventory must be
/// debited at the same one. Blending inflow against the full `m_old` and letting
/// the total mass update absorb the outflow separately is the natural-looking
/// alternative, and it debits the outflow at the END-of-tick composition instead:
/// total mass still balances exactly while the per-component books are off by
/// `ṁ_out·dt·(f_new − f_old)` every tick. I7 is what catches it.
///
/// The weights sum to `m_old + (ṁ_in − ṁ_out)·dt`, which is the new mass — so
/// normalizing them divides by the very inventory these fractions describe.
///
/// Shared by the tank and the capacitive vessel deliberately: it is the same
/// balance over a different substance, and two copies would be free to drift on
/// the one subtlety above. Callers skip it entirely when nothing arrived, rather
/// than passing a zero inflow — the fractions cannot have moved, and
/// re-normalizing `f·k` would add a rounding error's worth of drift on every tick
/// a holdup merely drains.
///
/// # Errors
/// `SimError::Numerical` if the weights do not form a valid composition.
pub fn blended_holdup_composition(
    current: &Composition,
    mass_old: f64,
    inflow_component_rate: &[f64],
    outflow_mass_rate: f64,
    dt: f64,
) -> Result<Composition, SimError> {
    let mut weights: Vec<f64> = inflow_component_rate.iter().map(|rate| rate * dt).collect();
    // A rounding guard since M24, not a licence to over-draw. A tank the solve
    // would have drawn past empty is STARVED and never reaches this function
    // (`starved_holdup_composition` debits it instead), and a wet tank's outflow
    // is at most its inventory by the rule that starves it (docs/DESIGN.md §28
    // fork 3), so `retained` can land below zero only by the rounding of the
    // product. A vessel has never come within 0.026 kg of empty (§28 fork 5).
    let retained = (mass_old - outflow_mass_rate * dt).max(0.0);
    for (weight, fraction) in weights.iter_mut().zip(current.fractions()) {
        *weight += retained * fraction;
    }
    Composition::from_weights(&weights)
}

/// A STARVED tank's end-of-tick component masses [kg]: what it held, plus what
/// arrived, minus what left AT WHAT LEFT (M24, docs/DESIGN.md §28 fork 4).
///
/// ```text
/// m_c_new = x_c,old·m_old + (ṁ_c,in − ṁ_c,out)·dt
/// ```
///
/// A wet tank's outflow leaves at its start-of-tick composition, which is why
/// `blended_holdup_composition` debits `f_c,old·ṁ_out`. A starved tank is swept
/// as a mixing point, so its outflow carries the MIX of what it held and what
/// the network fed it, and debiting it at `f_c,old` would book a different
/// fluid from the one the far end was credited with. `outflow_component_rate`
/// is summed off the outflow edges' own resolved composition, so the two sides
/// of every edge see one stream.
///
/// What remains is the inflow the solve does not see — a column draw or a
/// received boil-off vent, written after it — plus the solve's own residual at
/// the tank. Returned as weights rather than a composition, because the caller
/// decides what a tank holding almost nothing is made of (below the thermal
/// floor it takes the pass-through composition instead) and bounds any
/// component that lands below zero.
pub fn starved_holdup_weights(
    current: &Composition,
    mass_old: f64,
    inflow_component_rate: &[f64],
    outflow_component_rate: &[f64],
    dt: f64,
) -> Vec<f64> {
    current
        .fractions()
        .iter()
        .zip(inflow_component_rate.iter().zip(outflow_component_rate))
        .map(|(fraction, (inflow, outflow))| fraction * mass_old + (inflow - outflow) * dt)
        .collect()
}

/// Heat exchanged with the surroundings [W], SIGNED: positive into the body.
///
/// Newton's law of cooling, in the lumped `UA` form (any heat transfer text):
///
/// ```text
/// Q_ambient = UA·(T_AMBIENT − T_body)
/// ```
///
/// This is NOT "heat loss", and the naming carries weight (docs/DESIGN.md §4a).
/// The driving force is a temperature DIFFERENCE, so the one term must heat a
/// body colder than ambient and cool one hotter — a one-directional loss would
/// be wrong for a chilled tank on a warm day, and with the `Cooler` and the
/// `HeatExchanger` in place that is a reachable plant state rather than a
/// hypothetical. The direction falls out of the subtraction, so there is no
/// `if colder` branch, no second code path, and no sign convention of its own:
/// the same move that made `heat_load` the sole owner of the duty sign.
///
/// Ambient is the global `T_AMBIENT`, the same constant the `Atmosphere` node is
/// pinned to. A body sitting in the outside world and the outside world itself
/// must agree on how warm it is; a per-unit ambient would let them disagree.
#[inline]
pub fn ambient_exchange(ua: WattPerKelvin, body_temperature: Kelvin) -> Watt {
    Watt(ua.value() * (T_AMBIENT.value() - body_temperature.value()))
}

/// `(1 − e^{−x})/x`, the mean of `e^{−xξ}` over `ξ ∈ [0,1]`, continuous at 0.
///
/// The weight the dissipation term carries in `pipe_outlet_temperature`: heat
/// released a fraction `ξ` along the pipe only gets `(1 − ξ)` of the pipe left to
/// leak back out to ambient, and averaging that over a uniform release is exactly
/// this function. `ψ(0) = 1` is the limit, not a special case — with no ambient
/// coupling every watt released reaches the outlet.
///
/// `expm1` rather than `1.0 - x.exp()`: at small `x` the latter cancels to a few
/// significant digits, and small `x` is the ordinary case (`UA = 0` on every
/// scenario shipped today).
#[inline]
fn exp_decay_mean(x: f64) -> f64 {
    if x == 0.0 {
        1.0
    } else {
        -(-x).exp_m1() / x
    }
}

/// Outlet temperature of a pipe that exchanges heat with ambient and dissipates
/// friction into its own stream [K].
///
/// The analytic plug-flow solution — integrating
/// `ṁ·cp·dT/dx = ua'·(T_AMBIENT − T) + φ'` along the pipe, the same Newton's-law
/// driving force as `ambient_exchange` plus a uniform frictional source, applied
/// to a body with throughput and no inventory. With `C = |ṁ|·cp` [W/K] the
/// capacity rate and `β = UA/C`:
///
/// ```text
/// T_out = T_AMBIENT + (T_in − T_AMBIENT)·exp(−β) + (Φ/C)·ψ(β),   ψ(β) = (1−e^{−β})/β
/// ```
///
/// The two terms are **coupled, not applied in sequence**: heat released partway
/// along the pipe has only the remainder of the pipe to leak back out through, so
/// it arrives weighted by `ψ(β) ≤ 1` rather than in full. Adding `Φ/C` on top of
/// the pure exponential would over-credit it, by `(Φ/C)(1 − ψ)` — first order in
/// `β`, hence exactly zero wherever `UA = 0` but not in general.
///
/// **Φ is assumed uniform along the edge, and that is a stated limitation.** A
/// valve's dissipation is really concentrated at its trim, which the
/// fold-at-source convention puts at this edge's INLET, where the fluid then has
/// the whole pipe to shed it: the concentrated-inlet answer is `(Φ/C)·e^{−β}`
/// against this uniform `(Φ/C)·ψ(β)`, a difference of `≈ (Φ/C)·β/2`. This cannot
/// be resolved at the seam as it stands — the solver reports ONE `Φ` per edge
/// (`HydraulicSolution::edge_dissipation`), so `core` cannot tell a valve's share
/// from the pipe wall's without a second field. The error is zero at `UA = 0`,
/// which is every scenario in the workspace today (docs/DESIGN.md §3a).
///
/// `Φ ≥ 0` always (it is `α·|Q|³`), so this term only ever HEATS, whichever way
/// the flow runs — friction has no direction to get wrong, unlike the ambient
/// term's driving difference.
///
/// Three details are load bearing (docs/DESIGN.md §4a):
///
/// - **`|ṁ|`, not signed `ṁ`.** The flow's sign already did its work upstream,
///   selecting which end is the inlet; the denominator is a capacity RATE, a
///   magnitude. A signed `ṁ` flips the exponent on reverse flow, turning decay
///   into growth — the pipe would run *away* from ambient, and only on reversed
///   edges, which the reference plants do not have.
/// - **Analytic, not Euler.** `exp` cannot cross ambient however large the
///   exponent grows, where a `1 − UA/(ṁ·cp)` step overshoots and then diverges.
///   The tank balance tolerates explicit Euler because its `UA·dt/(m·cp)` is
///   ~1e-6 at refinery scale; here the denominator is a FLOW rather than an
///   inventory and a nearly-closed valve drives it toward zero, so the unstable
///   regime is one throttle away rather than unreachable.
/// - **`UA = 0` needs no special case.** `exp(0) = 1` makes the transform the
///   identity, which is what keeps every pre-existing scenario bit-identical.
///   Short-circuiting it with an `if ua == 0.0` would be worse than redundant:
///   it would mask the zero-flow guard below on exactly the case that reaches
///   it, leaving the guard untested and its falsification vacuous.
///
/// Zero flow is guarded, and only zero. At `ṁ = 0` with `UA = 0` the exponent is
/// `0/0 → NaN`, and a NaN temperature slips past the mixing guard to surface as
/// a bare non-finite with no diagnostic naming the pipe; a closed valve produces
/// such edges today. Every OTHER input is already well behaved — `ṁ = 0` with
/// `UA > 0` gives `−∞ → exp = 0 → T_AMBIENT`, and any nonzero `ṁ`, however
/// small, stays finite — so the test is exact equality, not a threshold. A
/// threshold would be a magic number that also silently flattened legitimately
/// small flows.
///
/// Returning the inlet at zero flow is the physically right answer here, not a
/// convenient one: a stagnant pipe carries no enthalpy either way, the same
/// reasoning that lets transport pick an arbitrary upwind end at exactly zero
/// flow. Modelling a stagnant pipe warming toward ambient needs pipe-wall
/// thermal mass, which is a fidelity step, not a guard. A stagnant edge also
/// dissipates nothing (`Φ = α·|Q|³ = 0`), so the guard cannot swallow a real
/// heat term.
#[inline]
pub fn pipe_outlet_temperature(
    inlet: Kelvin,
    ua: WattPerKelvin,
    mass_flow: KgPerSec,
    cp: JPerKgK,
    dissipation: Watt,
) -> Kelvin {
    let capacity_rate = mass_flow.value().abs() * cp.value(); // [W/K]
    if capacity_rate == 0.0 {
        return inlet;
    }
    let beta = ua.value() / capacity_rate;
    let decay = (-beta).exp();
    let friction_rise = dissipation.value() / capacity_rate * exp_decay_mean(beta);
    Kelvin(T_AMBIENT.value() + (inlet.value() - T_AMBIENT.value()) * decay + friction_rise)
}

/// The composition the sweep has already mixed for `node`.
///
/// A zero-volume node's composition is settled — by `mix_compositions` — before
/// its temperature is, and every consumer of `EnthalpyModel::mix_temperature`
/// runs after that. Looked up rather than recomputed, so the shape a mix is
/// inverted through is the same one the mass balance produced; returned as an
/// error rather than indexed, so a sweep-ordering bug cannot panic (rule 5).
fn mixed_composition_at(
    composition: &BTreeMap<NodeId, Composition>,
    node: NodeId,
) -> Result<&Composition, SimError> {
    composition.get(&node).ok_or_else(|| {
        SimError::Numerical(format!(
            "internal: node {node:?} was mixed for temperature before its composition"
        ))
    })
}

/// The end of `edge` the fluid comes FROM, given a signed mass flow.
///
/// Upwind by flow SIGN, never by edge direction — the donor-cell rule transport
/// uses, so reverse flow needs no special case. At exactly zero flow the pick is
/// arbitrary (nothing moves either way) and takes `from` to stay deterministic.
///
/// Extracted because temperature and composition are transported by the same
/// rule and must never disagree about which way a pipe runs: one definition, two
/// readers.
pub fn upwind_end(graph: &PlantGraph, edge: EdgeId, mass_flow: f64) -> NodeId {
    let (from, to) = graph.endpoints(edge);
    if mass_flow >= 0.0 {
        from
    } else {
        to
    }
}

/// The composition crossing `node`'s boundary along `edge`.
///
/// The composition analogue of `edge_temperature_at`, and — for every node but a
/// column — deliberately simpler: a pipe transforms the TEMPERATURE of what
/// passes through it (ambient exchange) but not its composition — heat crosses
/// the wall, mass does not — so there is no inlet/outlet distinction and both
/// ends see the upwind node's composition.
///
/// A **column is the exception**: it is the first node whose outlets do NOT all
/// carry its own composition. When the upwind node is a `Column` and this edge is
/// one of its draws, the crossing composition is that draw's *cut* composition,
/// read out of `separations` — the single pass the `SeparationModel` made over
/// this column when the sweep resolved it. Which draw an edge is comes from
/// `column_draw_at`, shared with `edge_temperature_at`'s matching arm so the two
/// fields of one draw can never come from two different draws. The result is
/// `Cow`: borrowed either way, from whichever map holds it.
///
/// **It reads a stored result rather than calling the model, and that is the
/// shape of the M7.1 seam.** Calling `separate` here would run it once per draw
/// edge per reader — the mistake the reactor note names explicitly for `react` —
/// which at the cut-point fidelity only wastes work, but at M7.3 would re-solve a
/// stage cascade four times over and give the column's two duties nowhere to
/// live. So the model runs once per column per tick in the sweep, and its result
/// travels to both consumers through `NodeStates`.
///
/// # Errors
/// `SimError::Numerical` if the upwind node's composition is unresolved, if a
/// column's separation is missing from `separations`, or if a column's draw
/// cannot be matched to this edge — all returned rather than indexed, so a
/// sweep-ordering bug cannot panic (rule 5).
pub fn edge_composition_at<'a>(
    graph: &PlantGraph,
    separations: &'a BTreeMap<NodeId, Separation>,
    composition: &'a BTreeMap<NodeId, Composition>,
    edge: EdgeId,
    mass_flow: f64,
) -> Result<Cow<'a, Composition>, SimError> {
    let upwind = upwind_end(graph, edge, mass_flow);
    let feed = composition.get(&upwind).ok_or_else(|| {
        SimError::Numerical(format!(
            "internal: upwind node '{}' of pipe '{}' unresolved during the composition sweep",
            graph.node(upwind).name,
            graph.pipe(edge).name
        ))
    })?;

    if let Some(cut) = column_draw_at(graph, separations, upwind, edge)? {
        return Ok(Cow::Borrowed(&cut.composition));
    }

    Ok(Cow::Borrowed(feed))
}

/// The separation result `edge` carries out of `upwind`, or `None` when `upwind`
/// is not a column.
///
/// **The single owner of "which draw am I", for both fields at once.** A draw
/// differs from its column in TWO ways — it carries a cut composition rather than
/// the feed mix, and (at the cascade fidelity) it leaves at its tray's temperature
/// rather than the column's mixed one — and both readers have to answer the same
/// question first: which of this column's draws is this edge? Two lookups would be
/// two copies of a rule that must agree, with nothing forcing them to; a mismatch
/// would hand a draw its own composition at a different draw's temperature, which
/// is not a state any plant can be in. That is the same rejection
/// `edge_temperature_at` makes for the two ENDS of a pipe, applied to the two
/// FIELDS of a draw.
///
/// # Errors
/// `SimError::Numerical` if the column's separation is missing from `separations`,
/// or if the edge cannot be matched to a draw — returned rather than indexed, so a
/// sweep-ordering bug cannot panic (rule 5).
fn column_draw_at<'a>(
    graph: &PlantGraph,
    separations: &'a BTreeMap<NodeId, Separation>,
    upwind: NodeId,
    edge: EdgeId,
) -> Result<Option<&'a DrawSeparation>, SimError> {
    let NodeKind::Column { draws, .. } = &graph.node(upwind).kind else {
        return Ok(None);
    };
    // The upwind node is a column, so this edge leaves it: it must be a draw.
    let draw = draw_index_for_edge(graph, edge, upwind, draws)?;
    let separation = separations.get(&upwind).ok_or_else(|| {
        SimError::Numerical(format!(
            "internal: column '{}' has no separation for this tick — the sweep \
             resolves every column before anything downstream of it reads a draw",
            graph.node(upwind).name
        ))
    })?;
    // Indexed through `get`, not `[]`: `separation.draws` is parallel to the
    // column's own draw list BY CONTRACT, and a trait contract held by an impl
    // in another crate is exactly what rule 5 says not to trust with a panic.
    let cut = separation.draws.get(draw).ok_or_else(|| {
        SimError::Numerical(format!(
            "separation model returned {} draws for column '{}', which has {}",
            separation.draws.len(),
            graph.node(upwind).name,
            draws.len()
        ))
    })?;
    Ok(Some(cut))
}

/// Which draw of `column` the `edge` leaving it feeds, by matching the edge's far
/// endpoint to a draw's outlet.
///
/// `column` is already known to be `edge`'s upwind end, so the *other* endpoint
/// is the draw's outlet. An edge leaving a column that names no draw is a wiring
/// error the loader is supposed to have rejected; it is an internal error here
/// rather than a panic (rule 5).
fn draw_index_for_edge(
    graph: &PlantGraph,
    edge: EdgeId,
    column: NodeId,
    draws: &[ColumnDraw],
) -> Result<usize, SimError> {
    let (from, to) = graph.endpoints(edge);
    let outlet = if from == column { to } else { from };
    draws
        .iter()
        .position(|d| d.outlet == outlet)
        .ok_or_else(|| {
            SimError::Numerical(format!(
                "internal: edge '{}' leaves column '{}' but its outlet '{}' names no draw",
                graph.pipe(edge).name,
                graph.node(column).name,
                graph.node(outlet).name
            ))
        })
}

/// The total mass entering `column` this tick [kg/s] — its feed.
///
/// The sum of its INFLOW edges, which is the feed alone: `network::edge_flows`
/// guards every draw edge to zero in the solve, and the real draw flows are
/// written back only after this sweep has run. So no draw can be counted as
/// feed here even when its stored direction runs into the column.
///
/// A column running BACKWARDS has no inflow at all and reports `0` rather than a
/// negative number. `Engine::tick` owns that refusal (its message names the
/// cause) and fires it after the sweep; this value only reaches a
/// `SeparationModel`, and the splitter does not read it.
fn column_feed_flow(
    graph: &PlantGraph,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    column: NodeId,
) -> KgPerSec {
    KgPerSec(
        mixed_inflow_edges(graph, edge_mass_flow, column)
            .iter()
            .map(|(_, _, into_node)| *into_node)
            .sum(),
    )
}

/// The temperature at which fluid crosses `node`'s boundary along `edge` [K].
///
/// The single owner of "which end of this edge am I looking at". Once a pipe
/// can exchange heat with ambient it has TWO temperatures, and every reader has
/// to resolve the same question — is this node the pipe's inlet or its outlet? —
/// with the same answer. Two call sites applying that rule independently is the
/// mistake the `HeatExchanger` note rejected in its "two independently computed
/// effectiveness terms" form: two copies of a rule that must agree, with nothing
/// forcing them to.
///
/// - `node` is the UPWIND end: the fluid leaves at the node's own temperature,
///   before the pipe has done anything to it. No transform.
/// - `node` is the DOWNSTREAM end: the fluid arrives at the transformed outlet.
///
/// A **column is the exception to "the node's own temperature"**, exactly as it is
/// for composition: its draws leave at whatever temperature the separation model
/// gave each of them — a tray's, under a cascade — and those differ from each
/// other and from the column's mixed feed. The pipe transform then runs from the
/// DRAW's temperature, not the column's. See `column_draw_at`.
///
/// Both readers pass every incident edge through here regardless of direction,
/// which is what makes the difference between the two ends land on the pipe
/// rather than on a node. Debiting a tank at its outflow pipe's OUTLET
/// temperature would charge the tank for heat the pipe traded with ambient
/// downstream of it — invisible while `UA = 0`, and a silent enthalpy error the
/// moment one pipe gets a real value.
///
/// With `UA = 0` both branches return the same number, which is why this change
/// is bit-identical to its predecessor by construction rather than by
/// measurement.
///
/// `dissipation` is that edge's frictional heat [W] from the solve, and it lands
/// on the same side of the same split: the upwind node is still at its own
/// temperature (the fluid has not been through the trim yet), and the downstream
/// end sees it. A valve therefore *reports* its inlet temperature, with its own
/// throttling heat appearing on the edge leaving it — the display consequence of
/// fold-at-source, stated rather than papered over (docs/DESIGN.md §3a).
///
/// # Errors
/// `SimError::Numerical` if the upwind node's temperature is not yet resolved —
/// returned rather than indexed so a sweep-ordering bug cannot panic (rule 5).
#[allow(clippy::too_many_arguments)]
pub fn edge_temperature_at(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy: &dyn EnthalpyModel,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    separations: &BTreeMap<NodeId, Separation>,
    edge: EdgeId,
    mass_flow: f64,
    dissipation: Watt,
    node: NodeId,
) -> Result<Kelvin, SimError> {
    let upwind = upwind_end(graph, edge, mass_flow);
    // A COLUMN's draws do not leave at the column's own mixed temperature: each
    // leaves at the temperature the separation model gives it, which at the
    // cascade fidelity is its tray's and differs per draw. This is the composition
    // arm's exact mirror, through the same lookup, and it is why a column is the
    // one node whose outlets disagree about BOTH fields.
    //
    // It sits here, above the `node == upwind` return, rather than inside it —
    // both ends need it. Fixing only the early return would leave the DOWNSTREAM
    // branch starting its ambient transform from the column's mixed feed
    // temperature, i.e. transforming the wrong inlet. That is invisible while
    // every `ambient_ua` is 0 and a silent enthalpy error the moment one is not —
    // the latent-bug class this function's own note above rejects, so the gate for
    // this arm puts a real `ambient_ua` on a draw pipe.
    let inlet = match column_draw_at(graph, separations, upwind, edge)? {
        Some(cut) => cut.temperature,
        None => temperature.get(&upwind).copied().ok_or_else(|| {
            SimError::Numerical(format!(
                "internal: upwind node '{}' of pipe '{}' unresolved during the temperature sweep",
                graph.node(upwind).name,
                graph.pipe(edge).name
            ))
        })?,
    };
    if node == upwind {
        return Ok(inlet);
    }
    let pipe = graph.pipe(edge);
    // The cp of what is IN the pipe this tick — the upwind node's resolved
    // composition, not the pipe's stored copy of last tick's. See
    // `stream_cp_at` for why the difference is not cosmetic.
    let cp = stream_cp_at(
        graph,
        slate,
        enthalpy,
        separations,
        composition,
        edge,
        mass_flow,
        inlet,
    )?;
    Ok(pipe_outlet_temperature(
        inlet,
        pipe.ambient_ua,
        KgPerSec(mass_flow),
        cp,
        dissipation,
    ))
}

/// The SPOT specific heat of the fluid crossing `edge` this tick [J/(kg·K)],
/// evaluated at `temperature`.
///
/// **Spot, and that answer is chosen rather than defaulted into** (§20's
/// consumer (b), which requires this slice to say which of two answers it took).
/// Its two readers both want a capacity at ONE state: `pipe_outlet_temperature`
/// puts it inside `β = UA/(ṁ·cp)` and inside `Φ/(ṁ·cp)`, and `inflow_totals`
/// puts it in the `C_min` an exchanger sizes its duty by. The mean the first of
/// those really wants would be over the pipe's own inlet-to-outlet path, which is
/// the quantity the formula is solving for; the exchanger's would be over an
/// outlet that does not exist until the duty is known. Both are circular, so both
/// take the inlet's spot value and the error is deferred with a trigger rather
/// than hidden behind a mean that looks more careful than it is.
///
/// Off the RESOLVED upwind composition, and deliberately not off
/// `pipe.stream.composition`. The stored copy is written at the end of a tick,
/// so every reader during the tick would see the PREVIOUS one's — and a stream
/// that has just changed composition would be charged the heat capacity of the
/// fluid it replaced.
///
/// That was shipped as an accepted one-tick lag on the grounds that no gate
/// could falsify it, which was true only while every multi-component test was
/// isothermal. It is not a bounded transient: a tank fed a cut whose cp differs
/// from the pipe's stale one books the wrong enthalpy on the very first tick,
/// and `a_tank_changing_composition_while_heating_lands_on_its_new_heat_capacity`
/// is the reference that says so.
///
/// Bit-identical on a one-component slate, where every composition is `[1.0]`
/// and both sources of cp agree by construction — which is what keeps every
/// M1/M2 golden unchanged.
///
/// # Errors
/// `SimError::Numerical` if the upwind node's composition is unresolved.
#[allow(clippy::too_many_arguments)]
pub fn stream_cp_at(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy: &dyn EnthalpyModel,
    separations: &BTreeMap<NodeId, Separation>,
    composition: &BTreeMap<NodeId, Composition>,
    edge: EdgeId,
    mass_flow: f64,
    temperature: Kelvin,
) -> Result<JPerKgK, SimError> {
    let crossing = edge_composition_at(graph, separations, composition, edge, mass_flow)?;
    enthalpy.spot_cp(slate, &crossing, temperature)
}

/// Total heat delivered into a node [W]: external heat, plus the operating duty
/// of a fired heater, minus that of a cooler, plus exchange with ambient.
///
/// This function is the single owner of the duty **sign convention**. Both
/// `Furnace` and `Cooler` store `duty` as a non-negative magnitude — "how much
/// heat this unit moves" — and which direction it moves is a property of the
/// unit, applied here. Nothing else in the engine needs to know.
///
/// The terms are separate fields and SUM here rather than sharing storage.
/// `heat_input` is the damage model's hook — `Command::SetHeatInput` sets it —
/// so folding a unit's duty into it would make a fire silently overwrite the
/// operator's setpoint instead of stacking on top of it. Every consumer of "how
/// much heat enters this node" goes through this function, so the two can never
/// drift apart. On a cooler that same sum gives the physically right answer for
/// free: a fire fights the cooling rather than replacing it.
///
/// A TANK's ambient exchange joins the same sum. It belongs here for the reason
/// the duty does — every consumer already asks this function how much heat
/// enters a node, so the tank's energy integration in `Engine::tick` picks the
/// term up without knowing it exists. Only a tank has one: an ambient boundary
/// needs a body with thermal mass and a temperature of its OWN, and a
/// zero-volume node has neither. A pipe does have one, but its outlet is not
/// its inlet plus `Q/(ṁ·cp)` — it needs the analytic plug-flow transform, which
/// is its own change to how transport works (docs/DESIGN.md §4a).
pub fn heat_load(node: &crate::graph::Node) -> Watt {
    let unit_term = match &node.kind {
        NodeKind::Furnace { duty, .. } => duty.value(),
        NodeKind::Cooler { duty } => -duty.value(),
        // The tank's temperature is its START-of-tick value here, which is what
        // makes this an explicit-Euler term like every other slow state.
        NodeKind::Tank(tank) => ambient_exchange(tank.ambient_ua, tank.temperature).value(),
        _ => 0.0,
    };
    Watt(node.heat_input.value() + unit_term)
}

/// Accept a computed temperature [K] only if physics permits it, or fail with
/// `context` explaining what produced the impossible value.
///
/// This is the single owner of "an absolute temperature below zero is an
/// error", for every path that computes one. There is more than one: a
/// zero-volume node mixes its inflows, a tank integrates its thermal
/// inventory, and each can be driven sub-zero by a large enough net heat
/// SINK. The result is FINITE in both cases, so the engine's NaN/Inf checks
/// never see it — without this guard the plant runs on a number physics
/// forbids, propagating downstream as an ordinary temperature.
///
/// Deliberately stated as "any net heat sink", not `if cooler` or
/// `if heat_input < 0`: a negative absolute temperature is broken whatever
/// produced it, and the callers owe nothing to which lever got them there.
///
/// Today exactly one lever reaches it — a cooler's duty, on the mixing path.
/// Nothing reaches the TANK path: `Command::SetHeatInput` refuses a negative
/// fire, a tank carries no duty of its own, and mixing cannot fall below its
/// coldest inflow. The tank guard is therefore cover held in advance, for the
/// ambient exchange that will put a signed Q straight onto that balance. That
/// is the point of a shared checker: the new term arrives already guarded,
/// instead of reopening this on whichever path is newest.
///
/// Err, never clamp. Clamping would report a plausible 0 K instead of the
/// temperature asked for, which is exactly the silently-wrong answer this
/// project treats as worse than a crash.
///
/// # Errors
/// `SimError::Numerical` if `value` is below absolute zero.
pub fn checked_temperature(
    value: f64,
    context: impl FnOnce() -> String,
) -> Result<Kelvin, SimError> {
    if value < 0.0 {
        return Err(SimError::Numerical(context()));
    }
    Ok(Kelvin(value))
}

/// True for nodes with no inventory, whose temperature is an instantaneous
/// mix of their inflows rather than a state (DESIGN §4a).
///
/// Pumps, valves, furnaces and coolers are zero-volume *pass-throughs*: with
/// exactly one inlet and one outlet (enforced by `validate_degrees`) the mixing
/// formula degenerates to "outlet temperature = inlet temperature", plus
/// whatever heat `heat_load` adds.
///
/// Pump work and valve throttling both dissipate into the stream as heat, and
/// since M5.1 they are MODELLED — but not here. A device folds into its outlet
/// EDGE (fold-at-source), so its friction is a term of that edge's outlet
/// transform (`pipe_outlet_temperature`), not of the node's mix. The visible
/// consequence, stated rather than papered over: a valve reports its INLET
/// temperature and its throttling heat appears on the pipe leaving it — the same
/// display choice §4a made for a pipe's two ends (docs/DESIGN.md §3a).
pub fn is_zero_volume(kind: &NodeKind) -> bool {
    matches!(
        kind,
        NodeKind::Junction
            | NodeKind::Pump { .. }
            | NodeKind::Valve { .. }
        // A relief valve is hydraulically a valve: zero-volume, 1-in-1-out, and
        // its temperature is the sweep's mix like any other pass-through.
        | NodeKind::ReliefValve { .. }
        | NodeKind::CheckValve { .. }
            | NodeKind::Furnace { .. }
            | NodeKind::Cooler { .. }
            | NodeKind::WaterCooler { .. }
            | NodeKind::CoilCell { .. }
            | NodeKind::HeatExchanger
            // A column holds no inventory: its composition is the instantaneous
            // feed mix and its draws separate that, never a holdup (DESIGN §5).
            // So it is swept in flow order like any zero-volume node — its own
            // temperature/composition are the mix of its ONE inflow (the feed);
            // the draws are outflows and do not enter that mix.
            | NodeKind::Column { .. }
            // A reactor holds no inventory either, so it is SWEPT — its
            // composition is its feed mix transformed by `react`. It is the one
            // exception to this function's "temperature = mix of inflows" premise:
            // its temperature is IMPOSED (`t_set`), not mixed. The sweep handles
            // that in its single-node branch; here it must count as zero-volume so
            // the sweep visits it at all (an inertial node would never resolve its
            // feed-dependent composition).
            | NodeKind::Reactor { .. }
            // A catalyst bed holds no FLUID inventory either (M57.1,
            // docs/DESIGN.md §62.1): its thermal mass is the bed, a state on the
            // node like a furnace's coil, so its fluid is swept like a furnace's.
            | NodeKind::BedReactor { .. }
            // A bed's hydrogen pass is zero-volume gas through the same bed
            // (M59.1, docs/DESIGN.md §64.2), swept with its bed as one vertex.
            | NodeKind::BedHydrogen { .. }
            // A compressor is a pass-through like a pump: its work is not
            // modelled as heat (§64.3, deferred).
            | NodeKind::Compressor { .. }
    )
}

/// Start-of-tick composition of an inertial node, or `None` for a zero-volume
/// node (whose composition must be mixed from its inflows instead).
///
/// The exact partition `boundary_temperature` makes, and it must stay exact:
/// the sweep seeds one field and mixes the other from the same `Some`/`None`
/// answer, so a node inertial in temperature and zero-volume in composition
/// would be swept in an order its own dependencies do not justify.
///
/// `Atmosphere` is the one node with no composition to state. It is given the
/// first slate component — arbitrary, and honestly so.
///
/// **That arbitrariness named its own expiry — "a decision the milestone that
/// back-feeds from a leak will have to make properly" — and M6.1 is that
/// milestone.** The decision made there was to REFUSE the back-feed rather than
/// to define the composition: `network::finalize` errors if a leak edge with a
/// nonzero area carries mass inward, which happens exactly when its junction is
/// below `P_ATM`. So this value is unreachable by construction rather than
/// merely untested, and the four options weighed to get there are priced in
/// docs/DESIGN.md §3b.
///
/// It stays a value and does not become an `Err` or an `unreachable!()`, which
/// would be a rule-5 violation reachable by the *undamaged* case: this function
/// runs for every inertial node on every tick, including in a plant whose
/// declared leak path is dormant and where nothing back-feeds at all. What
/// un-defers it properly is a slate carrying an air-like component and a reason
/// to burn it (DESIGN §3b, "what M6 does not attempt").
pub fn boundary_composition(kind: &NodeKind, slate: &Slate) -> Option<Composition> {
    match kind {
        NodeKind::Source { composition, .. } => Some(composition.clone()),
        NodeKind::Sink { composition, .. } => Some(composition.clone()),
        NodeKind::Atmosphere => Some(Composition::pure(slate.len(), 0)),
        NodeKind::Tank(tank) => Some(tank.composition.clone()),
        // A capacitive vessel is inertial in both fields, exactly like a tank:
        // it holds an inventory whose composition its outflows carry.
        NodeKind::Vessel(vessel) => Some(vessel.composition.clone()),
        NodeKind::Junction
        | NodeKind::Pump { .. }
        | NodeKind::Valve { .. }
        // A relief valve is hydraulically a valve: zero-volume, 1-in-1-out, and
        // its temperature is the sweep's mix like any other pass-through.
        | NodeKind::ReliefValve { .. }
        | NodeKind::CheckValve { .. }
        | NodeKind::Furnace { .. }
        | NodeKind::Cooler { .. }
        | NodeKind::WaterCooler { .. }
        | NodeKind::CoilCell { .. }
        | NodeKind::HeatExchanger
        | NodeKind::Column { .. }
        // A reactor's composition is its feed mix, then `react`'d — mixed here,
        // transformed in the sweep. It is zero-volume in BOTH fields (the
        // partition must agree), so it is `None` here too.
        | NodeKind::Reactor { .. }
        // A bed's composition is its feed mix, reacted in the sweep, like a
        // reactor's (M57.1, docs/DESIGN.md §62).
        | NodeKind::BedReactor { .. }
        // A hydrogen pass mixes its inflow and passes it on unreacted: the
        // hydrogen a bed consumes is a mass term, not a composition change
        // (M59, docs/DESIGN.md §64.6). A compressor passes its inflow on.
        | NodeKind::BedHydrogen { .. }
        | NodeKind::Compressor { .. } => None,
    }
}

/// Start-of-tick temperature of an inertial node, or `None` for a zero-volume
/// node (whose temperature must be mixed from its inflows instead).
///
/// `Atmosphere` is pinned to `T_AMBIENT`, symmetric with its pressure being
/// pinned to `P_ATM`: it is the outside world, not a modeled inventory.
pub fn boundary_temperature(kind: &NodeKind) -> Option<Kelvin> {
    match kind {
        NodeKind::Source { temperature, .. } => Some(*temperature),
        NodeKind::Sink { temperature, .. } => Some(*temperature),
        NodeKind::Atmosphere => Some(T_AMBIENT),
        NodeKind::Tank(tank) => Some(tank.temperature),
        // Inertial: its start-of-tick temperature, which is also what
        // `network::compile_edge` evaluates an outflow edge's gas density at and
        // what `VesselState::capacitance` is stated at. One value, three readers.
        NodeKind::Vessel(vessel) => Some(vessel.temperature),
        // Furnaces and coolers belong here, with the other zero-volume nodes:
        // `None` is what makes the sweep MIX their inflows and apply
        // `heat_load`. Returning `Some(..)` would compile and quietly make one
        // inertial — its duty would never reach the stream.
        NodeKind::Junction
        | NodeKind::Pump { .. }
        | NodeKind::Valve { .. }
        // A relief valve is hydraulically a valve: zero-volume, 1-in-1-out, and
        // its temperature is the sweep's mix like any other pass-through.
        | NodeKind::ReliefValve { .. }
        | NodeKind::CheckValve { .. }
        | NodeKind::Furnace { .. }
        | NodeKind::Cooler { .. }
        | NodeKind::WaterCooler { .. }
        | NodeKind::CoilCell { .. }
        | NodeKind::HeatExchanger
        // A column's temperature is its feed mix — the draws leave at the feed
        // temperature (DESIGN §5), which falls out for free: a zero-volume node
        // with one inflow mixes to that inflow's temperature, and the draws read
        // the column as their upwind end. No column-specific temperature code.
        | NodeKind::Column { .. }
        // A reactor's temperature is IMPOSED (`t_set`), not mixed — but it is
        // still `None` here, because seeding it as inertial would trip the
        // partition assertion (its composition is zero-volume) and the sweep must
        // visit it to resolve that composition. `t_set` is applied in the sweep's
        // single-node branch, overriding the mixed value.
        | NodeKind::Reactor { .. }
        // A bed's fluid leaves at the bed's temperature, a state the sweep
        // integrates in its single-node branch (`bed_reactor`), as a furnace's
        // leaves through its coil: `None` so the sweep visits it.
        | NodeKind::BedReactor { .. }
        // A hydrogen pass leaves at its bed's temperature, set with the bed in
        // the sweep's paired branch (M59.1, docs/DESIGN.md §64.2).
        | NodeKind::BedHydrogen { .. }
        | NodeKind::Compressor { .. } => None,
    }
}

/// The friction power [W] the solve reported for `edge`.
///
/// Mirrors how every reader here takes `edge_mass_flow` — `.get().unwrap_or(0)`
/// — so the two per-edge fields of one solve are looked up the same way and
/// cannot disagree about an edge the solve did not name. In practice neither
/// fallback is reachable: `Engine::tick` refuses a solution that omits any edge
/// from either map, so a missing key is a solver bug caught with the edge named,
/// not a silently zeroed heat term.
#[inline]
fn dissipation_on(edge_dissipation: &BTreeMap<EdgeId, Watt>, edge: EdgeId) -> Watt {
    edge_dissipation.get(&edge).copied().unwrap_or(Watt::ZERO)
}

/// Edges carrying flow *into* `node`, as `(edge, upstream node, ṁ into node)`.
///
/// The single definition of "inflow" — the dependency count and the mixing sum
/// both call it, so they cannot drift apart and leave the topological sort
/// waiting on an edge the mix never reads (or vice versa). The one inflow a
/// node waits on and does not mix is a column's steam inlet, which the column
/// reads on its own (`mixed_inflow_edges`, M62.6).
///
/// Self-loops are skipped: a pipe from a node to itself transports nothing
/// anywhere, and admitting it would make the node its own upstream dependency.
fn inflow_edges(
    graph: &PlantGraph,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    node: NodeId,
) -> Vec<(EdgeId, NodeId, f64)> {
    let mut inflows = Vec::new();
    // `incident` is sorted by edge id, so the sum order below is deterministic.
    for (edge, other, incoming) in graph.incident(node) {
        if other == node {
            continue;
        }
        let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
        let into_node = if incoming { flow } else { -flow };
        if into_node > 0.0 {
            inflows.push((edge, other, into_node));
        }
    }
    inflows
}

/// The inflows a node MIXES: `inflow_edges` but a column's steam inlet (M62.6,
/// docs/DESIGN.md §67.18), which enters its own stage and is never part of the
/// column's feed. The dependency count still waits on the steam (it reads
/// `inflow_edges`), so the column is resolved after its steam line is; every
/// node without a steam inlet mixes exactly what it always did.
fn mixed_inflow_edges(
    graph: &PlantGraph,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    node: NodeId,
) -> Vec<(EdgeId, NodeId, f64)> {
    let mut inflows = inflow_edges(graph, edge_mass_flow, node);
    if let Some(inlet) = column_steam_inlet(graph, node) {
        inflows.retain(|(edge, _, _)| *edge != inlet.edge);
    }
    inflows
}

/// `node`'s steam inlet, when it is a cascade column that declares one.
pub(crate) fn column_steam_inlet(graph: &PlantGraph, node: NodeId) -> Option<SteamInlet> {
    match &graph.node(node).kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec.steam,
        _ => None,
    }
}

/// The steam reaching `column` on its steam inlet this tick (M62.6,
/// docs/DESIGN.md §67.18): its rate, and its liquid-equivalent temperature — its
/// pipe's outlet enthalpy, the latent heat its upwind vapour carries included,
/// mixed as `mix_inflows` mixes one inflow. So the column's stage balance takes
/// the steam's enthalpy as the plant's books carry it.
///
/// Refused: a steam line running BACKWARDS (the column's pressure above the
/// supply's), which would draw from the column a flow no draw accounts for; and
/// a stream that is not pure free water. A line with no flow is a zero rate at a
/// placeholder temperature, the column's feed's, which nothing reads.
#[allow(clippy::too_many_arguments)] // one argument per map the mix reads
#[allow(clippy::type_complexity)]
fn steam_into_column(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy: &dyn EnthalpyModel,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    edge_dissipation: &BTreeMap<EdgeId, Watt>,
    (temperature, composition, separations, vapour): (
        &BTreeMap<NodeId, Kelvin>,
        &BTreeMap<NodeId, Composition>,
        &BTreeMap<NodeId, Separation>,
        &BTreeMap<NodeId, VapourShare>,
    ),
    column: NodeId,
    inlet: SteamInlet,
    placeholder: Kelvin,
) -> Result<ColumnSteam, SimError> {
    let edge = inlet.edge;
    let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
    let (from, to) = graph.endpoints(edge);
    let into = if to == column { flow } else { -flow };
    let upstream = if to == column { from } else { to };
    if !into.is_finite() || into < 0.0 {
        return Err(SimError::Numerical(format!(
            "the steam line '{}' into column '{}' runs backwards ({into:.4e} kg/s): the \
             column's pressure is above what reaches it from the steam supply, so the line \
             would draw from the column a flow none of its draws accounts for. Raise the \
             supply's pressure or close its valve (docs/DESIGN.md §67.18).",
            graph.pipe(edge).name,
            graph.node(column).name
        )));
    }
    if into == 0.0 {
        return Ok(ColumnSteam {
            stage: inlet.stage,
            flow: KgPerSec(0.0),
            temperature: placeholder,
        });
    }
    let crossing = edge_composition_at(graph, separations, composition, edge, flow)?;
    let water = slate.free_water();
    let pure = water.is_some_and(|w| crossing.fractions()[w] == 1.0);
    if !pure {
        return Err(SimError::Numerical(format!(
            "the steam line '{}' into column '{}' carries something other than pure water: \
             a steam inlet's water joins the column's water balance, and nothing else may \
             enter that way (docs/DESIGN.md §67.18).",
            graph.pipe(edge).name,
            graph.node(column).name
        )));
    }
    let inlet_t = edge_temperature_at(
        graph,
        slate,
        enthalpy,
        temperature,
        composition,
        separations,
        edge,
        flow,
        dissipation_on(edge_dissipation, edge),
        column,
    )?;
    let sensible = enthalpy
        .enthalpy_flux(slate, &crossing, KgPerSec(into), inlet_t)?
        .value();
    let latent = vapour
        .get(&upstream)
        .map_or(0.0, |share| into * share.latent.value());
    let capacity = into * enthalpy.spot_cp(slate, &crossing, inlet_t)?.value();
    let liquid_equivalent = enthalpy.mix_temperature(
        slate,
        &crossing,
        InflowEnthalpy {
            enthalpy_rate: sensible + latent,
            mass_rate: into,
            capacity_rate: capacity,
        },
    )?;
    Ok(ColumnSteam {
        stage: inlet.stage,
        flow: KgPerSec(into),
        temperature: liquid_equivalent,
    })
}

/// Every node's resolved intensive state for one tick.
///
/// Temperature and composition are resolved TOGETHER, on one sweep, because
/// they are transported by the same topology: the same inflow edges, the same
/// upwind rule, the same Kahn ordering, the same recycle rejection. A second
/// sweep computing composition alongside would be a duplicate of all four, free
/// to drift.
///
/// They are nonetheless mixed with DIFFERENT weights, which is the whole subtle
/// point of this pair — see `mix_compositions`.
#[derive(Debug, Clone, Default)]
pub struct NodeStates {
    /// Resolved temperature [K] per node.
    pub temperature: BTreeMap<NodeId, Kelvin>,
    /// Resolved mass-fraction composition per node.
    pub composition: BTreeMap<NodeId, Composition>,
    /// Per-reactor heat duties [W], the one extensive quantity resolved on this
    /// sweep. Computed where the reactor's inlet meets its imposed outlet (the
    /// only place feed, `T_in`, products and `Δh_rxn` are all in hand). Empty for
    /// a network with no reactor. See `ReactorDuty` and `reactor_duty`.
    pub reactor_duty: BTreeMap<NodeId, ReactorDuty>,
    /// Per-column separation: one `SeparationModel` pass per column per tick,
    /// made where the sweep resolves that column. Empty for a network with no
    /// column.
    ///
    /// **The result travels; the model does not.** `SeparationModel` reaches
    /// exactly one call site — this sweep, the way `ReactionModel` does — and its
    /// two consumers read it from here: `edge_composition_at` for which draw
    /// carries what, and `Engine::tick`'s post-sweep write for how much each draw
    /// carries. That is what forces the flow split and the composition split to
    /// come from the SAME pass, which per-component conservation at a zero-volume
    /// column requires (DESIGN §5).
    ///
    /// It is also where a cascade's condenser and reboiler duties will live, next
    /// to `reactor_duty` and for the same reason: an emergent diagnostic of a
    /// unit, resolved on the sweep that has the state to compute it.
    pub column_separation: BTreeMap<NodeId, Separation>,
    /// The zero-volume nodes whose temperature was HELD this tick rather than
    /// resolved: they had no inflow, so `mix_inflows` gave them the last value
    /// (or `T_AMBIENT`) as a placeholder (M19, docs/DESIGN.md §23 fork 4).
    ///
    /// Their entry in `temperature` stays, because the next tick's solve reads
    /// it and a hole there would be a different engine. What this set says is
    /// that the entry is not a MEASUREMENT: nothing computed it this tick. Read
    /// by exactly one consumer, `PlantGraph::measure`, which returns no
    /// measurement for a node in it — so a loop on a stagnant outlet holds rather
    /// than acting on, or seeding against, a number the sweep did not compute.
    ///
    /// **Never a furnace, since M34** (docs/DESIGN.md §37): a furnace with no
    /// inflow reports its coil's temperature, which IS computed — the coil is a
    /// state, and it took the tick's duty.
    pub held: BTreeSet<NodeId>,
    /// Each furnace's coil temperature [K] at the END of this tick (M34,
    /// docs/DESIGN.md §37), integrated on this sweep because it is the one site
    /// with the coil's inlet in hand. A RESULT, not a state: `Engine::tick`
    /// writes it back to `FurnaceCoil::temperature`, which is where it is read
    /// from. Empty on a plant with no furnace.
    pub coil_temperature: BTreeMap<NodeId, Kelvin>,
    /// Each furnace's stack loss [W], the tick's average (M36,
    /// docs/DESIGN.md §40): the share of its duty the coil did not absorb. A
    /// DIAGNOSTIC like `reactor_duty` — nothing in the forward solve reads it —
    /// published because a plant's energy books do not close without it.
    pub flue_loss: BTreeMap<NodeId, Watt>,
    /// Each furnace's burn-out fire [W] this tick (M37, docs/DESIGN.md §42):
    /// what leaked out of its burst tubes, times the fluid's heating value,
    /// burning in its firebox. Zero on intact tubes and on a patched hole. A
    /// DIAGNOSTIC like `flue_loss`, and for the same reason: the books do not
    /// close without it. Empty on a plant with no furnace.
    pub tube_fire: BTreeMap<NodeId, Watt>,
    /// What this tick did to each catalyst bed (M57.1, docs/DESIGN.md §62.1),
    /// integrated on this sweep because it is the one site with the bed's feed
    /// in hand. Its end-of-tick temperature is a RESULT, not a state:
    /// `Engine::tick` writes it back to `CatalystBed::temperature`, as it does
    /// a furnace's coil. Empty on a plant with no bed.
    pub bed: BTreeMap<NodeId, BedPass>,
    /// Each node whose stream is partly vapour this tick, and its vapour (M53,
    /// docs/DESIGN.md §58): a supply above its bubble point, and a zero-volume
    /// node whose line flash boiled it. Empty on every plant that does not
    /// select `[fidelity] line_flash = "equilibrium"`.
    ///
    /// Read by three consumers, and the vapour every one of them reads is the
    /// UPWIND node's: the sweep's own mix (the latent heat arriving at the next
    /// node), `Engine::tick`'s edge write (`Stream::vapour_fraction` and
    /// `latent`) and its tank balance, and the next tick's flow solve, which
    /// re-flashes the node at each trial pressure from `liquid_equivalent`.
    pub vapour: BTreeMap<NodeId, VapourShare>,
    /// What each tank holding free water sends out of its outlets this tick:
    /// its bottom layer, water first, over the tick's whole draw (M62.3,
    /// docs/DESIGN.md §67.3; `TankState::drawn_composition`). The same
    /// composition the sweep seeded in `composition` for that tank, kept apart
    /// because the next solve reads it for its outlets' densities
    /// (`network::carried_composition`) where every other tank's inventory is
    /// read straight off the graph. Empty on every plant whose slate has no
    /// free-water member.
    pub tank_draw: BTreeMap<NodeId, Composition>,
    /// What each water cooler did this tick (M62.4b, docs/DESIGN.md §67.13):
    /// its duty and its cooling water's outlet. Empty on a plant without one.
    pub water_cooler: BTreeMap<NodeId, WaterCoolerPass>,
}

/// What one tick does at a water cooler (M62.4b, docs/DESIGN.md §67.13).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaterCoolerPass {
    /// Heat taken from the stream into the cooling water [W]; negative where the
    /// water is the warmer of the two inlets and heats the stream.
    pub duty: Watt,
    /// The cooling water's outlet temperature [K], `T_in + Q/(ṁ_w·cp_w)`.
    pub water_outlet: Kelvin,
}

/// Cooling water's heat capacity [J/(kg·K)]: liquid water near 30 °C, held
/// constant (§67.13) — the utility is a liquid that does not boil.
pub const COOLING_WATER_CP: f64 = 4186.0;

/// The cooling water's boiling point at atmospheric pressure [K]: a water
/// cooler whose water would leave at or past it halts the run (§67.13).
pub const COOLING_WATER_BOILING: Kelvin = Kelvin(373.15);

/// Equal-duty zones a water cooler's condensing side is rated over (§67.13,
/// Kern's zone analysis). Probed: see docs/DESIGN.md §67.13's build note.
pub const WATER_COOLER_ZONES: usize = 10;

/// Evaluations the zone method's duty search may spend: a backstop past which
/// it is an `Err`. Its bracket halves at least once every two steps.
const WATER_COOLER_MAX_EVALUATIONS: u32 = 200;

/// A swept node's temperature from its LIQUID-EQUIVALENT mix, settled by the line
/// flash at the node's own solved pressure (M53, docs/DESIGN.md §58 fork 2); its
/// vapour, if any, recorded in `vapour`. `NoLineFlash` hands the mix back
/// untouched and records nothing.
#[allow(clippy::too_many_arguments)] // one argument per input the settle reads
fn settle_node(
    graph: &PlantGraph,
    slate: &Slate,
    (line_flash, thermo, enthalpy): (&dyn LineFlashModel, &dyn ThermoModel, &dyn EnthalpyModel),
    node_pressure: &BTreeMap<NodeId, Pascal>,
    composition: &BTreeMap<NodeId, Composition>,
    vapour: &mut BTreeMap<NodeId, VapourShare>,
    id: NodeId,
    liquid_equivalent: Kelvin,
) -> Result<Kelvin, SimError> {
    // A model that carries no vapour hands every mix back as it came: answered
    // before the lookups, because this runs at every swept node every tick.
    if !line_flash.carries_vapour() {
        return Ok(liquid_equivalent);
    }
    let pressure = node_pressure.get(&id).copied().ok_or_else(|| {
        SimError::Numerical(format!(
            "internal: node '{}' has no solved pressure for its line flash",
            graph.node(id).name
        ))
    })?;
    let settled = line_flash.settle(
        slate,
        mixed_composition_at(composition, id)?,
        liquid_equivalent,
        pressure,
        thermo,
        enthalpy,
    )?;
    if let Some(share) = settled.vapour {
        vapour.insert(id, share);
    }
    Ok(settled.temperature)
}

/// Σ ṁ·latent [W] arriving at `node` on its inflow edges — the latent heat its
/// upwind nodes' vapour carries in (M53, docs/DESIGN.md §58). `None`, not
/// `Some(0.0)`, when no upwind node has vapour, so a caller adds nothing at all
/// to its sum and every plant without the model keeps its bits.
pub(crate) fn inflow_latent(
    graph: &PlantGraph,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    vapour: &BTreeMap<NodeId, VapourShare>,
    node: NodeId,
) -> Option<f64> {
    if vapour.is_empty() {
        return None;
    }
    let mut total = None;
    for (_edge, upstream, into_node) in mixed_inflow_edges(graph, edge_mass_flow, node) {
        if let Some(share) = vapour.get(&upstream) {
            *total.get_or_insert(0.0) += into_node * share.latent.value();
        }
    }
    total
}

/// What one tick does to a catalyst bed (M57.1, docs/DESIGN.md §62.1).
#[derive(Debug, Clone, Copy)]
pub struct BedPass {
    /// The bed's temperature at the END of the tick [K].
    pub temperature: Kelvin,
    /// Heat the reaction released into the bed this tick [W],
    /// `Q = −ṁ·Δh_rxn`: positive when it heats the bed, negative when it
    /// absorbs. A DIAGNOSTIC like `flue_loss`: a plant's energy books do not
    /// close without it, since the sensible datum does not track chemistry.
    pub reaction_heat: Watt,
    /// The residence time the chemistry ran over [s], `τ = V·ρ_feed/ṁ`
    /// (§62.2). `None` at zero flow, where `τ` is infinite.
    pub residence_time: Option<Seconds>,
    /// The fire a failed wall's leak feeds this tick [W] (M58, docs/DESIGN.md
    /// §63.3): what flowed out of the hole times the heating value, all of
    /// it; `BedWall::fire_share` of it heated the bed. Zero on an intact wall
    /// and on a patched hole. A DIAGNOSTIC like `reaction_heat`.
    pub wall_fire: Watt,
    /// The bed's hydrogen this tick (M59.1, docs/DESIGN.md §64), `None` on a
    /// bed without a hydrogen pass.
    pub hydrogen: Option<HydrogenPass>,
}

/// What a bed's hydrogen did this tick (M59.1, docs/DESIGN.md §64).
#[derive(Debug, Clone, Copy)]
pub struct HydrogenPass {
    /// The pass node.
    pub pass: NodeId,
    /// The factor every rate ran at, `f = (p/p_ref)^n` (dimensionless).
    pub factor: f64,
    /// The heat the hydrogen carried INTO the bed this tick [W],
    /// `H_in,H2 − H_out,H2`: negative while it cools the bed. A DIAGNOSTIC like
    /// `reaction_heat`.
    pub heat: Watt,
    /// What the reaction consumed of it (M59.2, docs/DESIGN.md §64.11), `None`
    /// on a pass that declares no consumption.
    pub consumption: Option<PassConsumption>,
}

/// The hydrogen a bed's reaction consumed from its pass this tick (M59.2,
/// docs/DESIGN.md §64.11).
#[derive(Debug, Clone, Copy)]
pub struct PassConsumption {
    /// What the gas side lost at the pass this tick [kg/s]: the pass's net
    /// inflow in this tick's solve, which drew `rate · P` there. The BOOKED
    /// figure — every gas-side ledger closes on it.
    pub consumed: KgPerSec,
    /// The sensible enthalpy the consumed hydrogen takes off the books [W]: it
    /// is heated through the bed with the rest of the hydrogen and consumed at
    /// the outlet temperature, so `consumed · h(T_out)`. A DIAGNOSTIC like
    /// `reaction_heat`, which the energy books need beside it.
    pub consumed_heat: Watt,
    /// What this tick's reaction draws [kg/s], `ṁ_oil · Σ Δyᵢ · uptakeᵢ`: the
    /// NEXT tick's sink, at this tick's pressure.
    pub demand: KgPerSec,
    /// The next tick's sink rate [kg/(s·Pa)], `demand / P` at the pressure this
    /// tick's reaction ran at; zero at zero pressure, where nothing reacts.
    pub rate: f64,
}

/// The two heat duties a reactor's isothermal setpoint implies, both extensive
/// [W]. Neither drives the forward solve — they are DIAGNOSTICS, computed so the
/// energy gate can pin each term separately (DESIGN §5). The reactor's outlet
/// temperature is `t_set` regardless of either.
#[derive(Debug, Clone, Copy)]
pub struct ReactorDuty {
    /// Emergent SENSIBLE duty [W]: the heat that carries the feed from its inlet
    /// mix enthalpy to the product enthalpy at `t_set`,
    /// `ṁ·cp_out·(T_set − T_REF) − Σ ṁ_in·cp_in·(T_in − T_REF)`. It INCLUDES the
    /// cp shift the composition change causes (`cp_out` is the products'). Closes
    /// by construction — the reactor imposes `t_set` like a furnace imposes duty.
    pub sensible: Watt,
    /// Reported PHYSICAL duty [W]: `sensible + ṁ·Δh_rxn`. The heat the reactor
    /// actually trades with the outside to hold `t_set` while running the
    /// reaction. The `Δh_rxn` term sits OUTSIDE the sensible-only datum, which is
    /// why I6 excludes reactors (DESIGN §5).
    pub reported: Watt,
}

/// The largest relative move, between the states a flow solve READ and the
/// states the sweep then resolved from that solve's flows, of the density a
/// stream leaving a zero-volume node flows at (M55.0, docs/DESIGN.md §60.0,
/// ledger row B50) — what the solve reads of such a node, at the pressure it
/// solved there: the line flash's mixture where the node carries vapour
/// (`network::two_phase_density`), else its composition's density at its
/// temperature (`network::liquid_density`; a liquid's ignores the
/// temperature).
///
/// Where the solve read no state of a node (the first tick), what it read was
/// the stored stream of each pipe at the node (`Pipe::stream`), as a liquid.
#[allow(clippy::too_many_arguments)] // one argument per seam the density reads
pub fn swept_density_change(
    graph: &PlantGraph,
    slate: &Slate,
    line_flash: &dyn LineFlashModel,
    thermo: &dyn ThermoModel,
    enthalpy: &dyn EnthalpyModel,
    node_pressure: &BTreeMap<NodeId, Pascal>,
    read: &NodeStates,
    resolved: &NodeStates,
) -> Result<f64, SimError> {
    let density = |states: &NodeStates,
                   id: NodeId,
                   pressure: Pascal|
     -> Result<Option<f64>, SimError> {
        let Some(composition) = states.composition.get(&id) else {
            return Ok(None);
        };
        let temperature = states
            .vapour
            .get(&id)
            .map(|share| share.liquid_equivalent)
            .or_else(|| states.temperature.get(&id).copied());
        let Some(temperature) = temperature else {
            return Ok(None);
        };
        if line_flash.carries_vapour() {
            if let Some(two_phase) =
                line_flash.density(slate, composition, temperature, pressure, thermo, enthalpy)?
            {
                return Ok(Some(two_phase.mixture.value()));
            }
        }
        Ok(Some(
            composition
                .density_at(slate, pressure, temperature)?
                .value(),
        ))
    };
    let mut largest = 0.0_f64;
    for id in graph.node_ids() {
        if !is_zero_volume(&graph.node(id).kind) {
            continue;
        }
        let Some(&pressure) = node_pressure.get(&id) else {
            continue;
        };
        let Some(after) = density(resolved, id, pressure)? else {
            continue;
        };
        let mut befores = Vec::new();
        match density(read, id, pressure)? {
            Some(before) => befores.push(before),
            // Not resolved when the solve read it (the first tick): the solve
            // read the stored stream of each pipe at the node instead — liquid,
            // at that stream's composition and temperature.
            None => {
                for (eid, _, _) in graph.incident(id) {
                    let stream = &graph.pipe(eid).stream;
                    befores.push(
                        stream
                            .composition
                            .density_at(slate, pressure, stream.temperature)?
                            .value(),
                    );
                }
            }
        }
        for before in befores {
            largest = largest.max((after - before).abs() / after.abs().max(f64::MIN_POSITIVE));
        }
    }
    Ok(largest)
}

/// Resolve every node's temperature [K] and composition for this tick.
///
/// Inertial nodes seed the field with their start-of-tick temperature;
/// zero-volume nodes are then swept in flow order, each mixing its inflows:
///
/// ```text
/// T_mix = T_REF + Σ(ṁ_in·cp_in·(T_in − T_REF)) / Σ(ṁ_in·cp_in)
/// ```
///
/// which is the enthalpy balance of a zero-volume mixing point (first law with
/// no accumulation and no work), reducing to the mass-weighted mean when every
/// stream shares a `cp`.
///
/// and composition alongside it, by mass alone:
///
/// ```text
/// f_c = Σ(ṁ_in · f_in,c) / Σ ṁ_in
/// ```
///
/// `previous` supplies the fallback for a zero-volume node with **no inflow**:
/// nothing enters it, so its temperature is physically indeterminate — and it
/// carries no enthalpy either way, since mass balance forces zero outflow too.
/// Holding the last resolved value keeps the field finite and reproducible
/// instead of dividing 0/0.
///
/// A **reactor** is the one zero-volume node whose two fields diverge: its
/// composition is the feed mix transformed by `reactions`, and its temperature
/// is IMPOSED (`t_set`) rather than mixed. Both, plus its emergent duties, are
/// resolved in the single-node branch (`reactor_duty`).
///
/// A **column** mixes both fields like any other zero-volume node, and then makes
/// one `SeparationModel` pass over the result — after its own temperature is
/// resolved, since the pass is a function of it. Nothing downstream of a column
/// is released until that pass is stored, which is what lets
/// `edge_composition_at` read it rather than recompute it.
///
/// A **starved tank** (M24, docs/DESIGN.md §28 fork 4) is swept as a mixing
/// point too, not seeded: its own inventory is one more inflow at its supply
/// rate, its own composition and its own temperature, so its outflow carries
/// the mix of what it held and what the network fed it. `starved` is the
/// solve's own list (`HydraulicSolution::starved`), empty on every tick that
/// starves nothing — and then this sweep is the pre-M24 one to the bit.
///
/// # Errors
/// `SimError::Numerical` if a recycle among zero-volume nodes leaves the sweep
/// with no valid order (see the module docs) — with its own message when the
/// loop runs through a starved tank — if `reactions` cannot produce valid
/// products for a reactor's feed, or if `separation` cannot split a column's
/// feed.
#[allow(clippy::too_many_arguments)]
pub fn resolve_node_states(
    graph: &PlantGraph,
    slate: &Slate,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    edge_dissipation: &BTreeMap<EdgeId, Watt>,
    reactions: &dyn ReactionModel,
    separation: &dyn SeparationModel,
    thermo: &dyn ThermoModel,
    enthalpy: &dyn EnthalpyModel,
    line_flash: &dyn LineFlashModel,
    node_pressure: &BTreeMap<NodeId, Pascal>,
    previous: &NodeStates,
    starved: &BTreeMap<NodeId, StarvedTank>,
    dt: Seconds,
) -> Result<NodeStates, SimError> {
    let mut temperature: BTreeMap<NodeId, Kelvin> = BTreeMap::new();
    let mut vapour: BTreeMap<NodeId, VapourShare> = BTreeMap::new();
    let mut coil_temperature: BTreeMap<NodeId, Kelvin> = BTreeMap::new();
    let mut flue_loss: BTreeMap<NodeId, Watt> = BTreeMap::new();
    let mut tube_fire: BTreeMap<NodeId, Watt> = BTreeMap::new();
    let mut bed: BTreeMap<NodeId, BedPass> = BTreeMap::new();
    let mut composition: BTreeMap<NodeId, Composition> = BTreeMap::new();
    let mut reactor_duties: BTreeMap<NodeId, ReactorDuty> = BTreeMap::new();
    let mut separations: BTreeMap<NodeId, Separation> = BTreeMap::new();
    let mut held: BTreeSet<NodeId> = BTreeSet::new();
    let mut tank_draw: BTreeMap<NodeId, Composition> = BTreeMap::new();
    let mut water_cooler: BTreeMap<NodeId, WaterCoolerPass> = BTreeMap::new();
    // Every draw LINE's pipes (M62.4b, docs/DESIGN.md §67.13): each node on one
    // depends on its line's pipe in by topology — the solve wrote it zero — and
    // the sweep's own flows carry the draw on each from the moment its column
    // separates. On every plant without one, nothing below reads these.
    let lines = graph.draw_lines();
    let line_edge = |edge: EdgeId| lines.iter().any(|(_, _, line)| line.contains(&edge));
    let mut flows: Cow<BTreeMap<EdgeId, f64>> = Cow::Borrowed(edge_mass_flow);

    // 1. Inertial nodes are the roots of the sweep: known before it starts.
    //    Both fields are seeded from the SAME node, so the two boundary helpers
    //    agreeing on which nodes are inertial is load-bearing — asserted here
    //    rather than assumed, because a mismatch would leave a node seeded in
    //    one field and swept in the other, and the sweep would then read an
    //    unresolved upwind and error somewhere far from the cause.
    let mut zero_volume: Vec<NodeId> = Vec::new();
    // "Swept rather than seeded", which since M24 is a property of the node's
    // STATE as well as its kind: a starved tank is a tank and is swept. One
    // closure, read at all three sites that ask (this partition, the dependency
    // count and the release below), so they cannot disagree about a dry tank.
    let swept = |id: NodeId| {
        is_zero_volume(&graph.node(id).kind)
            || (starved.contains_key(&id) && matches!(graph.node(id).kind, NodeKind::Tank(_)))
    };
    for id in graph.node_ids() {
        let kind = &graph.node(id).kind;
        if swept(id) && !is_zero_volume(kind) {
            zero_volume.push(id);
            continue;
        }
        match (
            boundary_temperature(kind),
            boundary_composition(kind, slate),
        ) {
            (Some(t), Some(c)) => {
                // A supply above its own bubble point boils where it stands, at
                // its declared state (M53, docs/DESIGN.md §58 fork 2). Only a
                // model that carries vapour answers; `NoLineFlash` never does.
                if let NodeKind::Source { pressure, .. } = kind {
                    if let Some(share) =
                        line_flash.supply(slate, &c, t, *pressure, thermo, enthalpy)?
                    {
                        vapour.insert(id, share);
                    }
                }
                // A tank holding free water sends its bottom layer out, not
                // its inventory (M62.3, docs/DESIGN.md §67.3): seeded HERE, so
                // every reader of an outlet — the mixes downstream, the
                // engine's books, the edge write — takes the one answer.
                let c = match kind {
                    NodeKind::Tank(tank) => {
                        let drawn = graph.solved_outflow(id, |edge| {
                            edge_mass_flow.get(&edge).copied().unwrap_or(0.0)
                        }) * dt.value();
                        match tank.drawn_composition(slate, Kg(drawn))? {
                            Some(draw) => {
                                tank_draw.insert(id, draw.clone());
                                draw
                            }
                            None => c,
                        }
                    }
                    _ => c,
                };
                temperature.insert(id, t);
                composition.insert(id, c);
            }
            (None, None) => zero_volume.push(id),
            _ => {
                return Err(SimError::Numerical(format!(
                    "internal: node '{}' is inertial in one intensive field and \
                     zero-volume in the other — `boundary_temperature` and \
                     `boundary_composition` must partition the node kinds identically",
                    graph.node(id).name
                )))
            }
        }
    }

    // 2. Group the zero-volume nodes into sweep VERTICES. Almost every vertex
    //    is a single node; a heat exchanger's two sides are ONE vertex.
    //
    //    This merge is the whole reason the sweep is not simply per-node. Each
    //    side's outlet depends on the OTHER side's inlet, which is not one of
    //    its own inflow edges — so a per-node sweep would happily mark side A
    //    ready as soon as A's own upstreams cleared and mix it against a stale
    //    partner inlet. That failure converges, serializes and reruns
    //    identically; nothing downstream looks wrong. Merged, a side becomes
    //    ready only when the UNION of both sides' upstreams is resolved, which
    //    is exactly the condition under which both inlets are known.
    //
    //    A vertex is keyed by its LEADER, the lower of the two node ids, so the
    //    key is a function of the graph alone (rule 3).
    //    A catalyst bed and its hydrogen pass are one vertex too (M59.1,
    //    docs/DESIGN.md §64.2), for the same reason: both leave at the bed's
    //    temperature, which both inflows set.
    //    A furnace and its coil's cells are one vertex too (M62.4c,
    //    docs/DESIGN.md §67.14), led by the furnace: they share one metal.
    let leader_of = |id: NodeId| -> NodeId {
        if let NodeKind::CoilCell { furnace } = graph.node(id).kind {
            return furnace;
        }
        match graph.sweep_partner(id) {
            Some(partner) => id.min(partner),
            None => id,
        }
    };
    let mut members: BTreeMap<NodeId, Vec<NodeId>> = BTreeMap::new();
    for &id in &zero_volume {
        members.entry(leader_of(id)).or_default().push(id);
    }

    // 3. Count each vertex's unresolved upstream dependencies. Only zero-volume
    //    upstreams count — an inertial upstream is already known.
    //
    //    A dependency on our OWN vertex IS counted here, and the release step
    //    below deliberately never clears it. That asymmetry is the cycle
    //    detector: one exchanger side feeding the other is a genuine circular
    //    definition — A's outlet depends on B's inlet, which is A's outlet — so
    //    the vertex must never become ready and must fall into the step-5
    //    recycle error. Discounting the self-dependency instead would make the
    //    vertex ready immediately and resolve it against a partner inlet that
    //    does not exist yet.
    let mut pending: BTreeMap<NodeId, usize> = BTreeMap::new();
    for (&leader, sides) in &members {
        let deps = sides
            .iter()
            .flat_map(|&id| inflow_edges(graph, edge_mass_flow, id))
            .filter(|(edge, upstream, _)| {
                swept(*upstream) && !line_edge(*edge) && !graph.is_coil_segment(*edge)
            })
            .count();
        // A draw line's node waits on its line's pipe in, whatever the solve
        // wrote on it (M62.4b).
        let line_deps = sides
            .iter()
            .flat_map(|&id| graph.incident(id))
            .filter(|&(edge, _, incoming)| incoming && line_edge(edge))
            .count();
        pending.insert(leader, deps + line_deps);
    }

    // 4. Kahn sweep. BTreeSet, always popping the lowest id: the order is then
    //    a function of the graph alone, never of insertion history (rule 3).
    //    Any valid topological order yields identical temperatures anyway —
    //    each mix reads only already-resolved upstreams — but a deterministic
    //    order keeps the float summation order fixed too.
    let mut ready: BTreeSet<NodeId> = members
        .keys()
        .copied()
        .filter(|leader| pending[leader] == 0)
        .collect();

    let mut resolved = 0usize;
    while let Some(&leader) = ready.iter().next() {
        ready.remove(&leader);
        let sides = &members[&leader];
        let edge_mass_flow: &BTreeMap<EdgeId, f64> = &flows;
        // The draws a column separated in this vertex, written on its lines'
        // pipes once the vertex is released (M62.4b).
        let mut line_writes: Vec<(EdgeId, f64)> = Vec::new();

        // A furnace whose coil has cells: the vertex in the coil's order, its
        // first cell's inflow mix every member's composition — nothing reacts
        // in a coil (M62.4c, docs/DESIGN.md §67.14).
        let coil = (sides.len() > 1)
            .then(|| {
                sides
                    .iter()
                    .copied()
                    .find(|&id| matches!(graph.node(id).kind, NodeKind::Furnace { .. }))
            })
            .flatten()
            .map(|furnace| graph.coil_chain(furnace));

        // Composition first, and uniformly across both branches: it is settled
        // by mass alone, so an exchanger's two sides are ordinary mixing points
        // to it. Heat crosses between them; mass does not, and a pair whose
        // compositions influenced each other would be modelling a leak.
        if let Some(chain) = &coil {
            let mixed = mix_compositions(
                graph,
                slate,
                edge_mass_flow,
                &separations,
                &composition,
                previous,
                chain[0],
            )?;
            for &id in chain {
                composition.insert(id, mixed.clone());
            }
        }
        for &id in sides.iter().filter(|_| coil.is_none()) {
            let mixed = match starved_tank(graph, starved, id) {
                Some((tank, supply)) => starved_tank_composition(
                    graph,
                    edge_mass_flow,
                    &separations,
                    &composition,
                    id,
                    &tank.composition,
                    supply,
                )?,
                None => mix_compositions(
                    graph,
                    slate,
                    edge_mass_flow,
                    &separations,
                    &composition,
                    previous,
                    id,
                )?,
            };
            composition.insert(id, mixed);
        }

        match sides.as_slice() {
            _ if coil.is_some() => {
                let chain = coil.as_deref().unwrap_or_default();
                let furnace = chain[chain.len() - 1];
                let NodeKind::Furnace {
                    duty,
                    coil: metal,
                    flame_temperature,
                    tubes,
                } = &graph.node(furnace).kind
                else {
                    return Err(SimError::Numerical(format!(
                        "internal: the coil ending at '{}' does not end at its furnace",
                        graph.node(furnace).name
                    )));
                };
                let pass = furnace_coil_cells(
                    graph,
                    slate,
                    (line_flash, thermo, enthalpy),
                    (edge_mass_flow, edge_dissipation),
                    (&mut temperature, &composition, &separations, &mut vapour),
                    node_pressure,
                    previous,
                    chain,
                    (*duty, metal, *flame_temperature, tubes),
                    dt,
                )?;
                coil_temperature.insert(furnace, pass.coil);
                flue_loss.insert(furnace, pass.flue);
                tube_fire.insert(furnace, pass.fire);
            }
            [a, b] if graph.exchanger_partner(*a).is_none() => {
                // A catalyst bed and its hydrogen pass (M59.1, docs/DESIGN.md
                // §64.2): the bed steps on both inflows, and both streams leave
                // at one temperature. The bed reads the pass's solved pressure.
                let (bed_id, pass_id) =
                    if matches!(graph.node(*a).kind, NodeKind::BedReactor { .. }) {
                        (*a, *b)
                    } else {
                        (*b, *a)
                    };
                let (
                    NodeKind::BedReactor {
                        bed: catalyst,
                        wall,
                    },
                    NodeKind::BedHydrogen {
                        kinetics,
                        consumption,
                        ..
                    },
                ) = (&graph.node(bed_id).kind, &graph.node(pass_id).kind)
                else {
                    return Err(SimError::Numerical(format!(
                        "internal: '{}' and '{}' were swept as one vertex, but are neither an \
                         exchanger's two sides nor a bed and its hydrogen pass",
                        graph.node(*a).name,
                        graph.node(*b).name
                    )));
                };
                let pressure = node_pressure.get(&pass_id).copied().ok_or_else(|| {
                    SimError::Numerical(format!(
                        "hydrogen pass '{}': the solve reported no pressure for it",
                        graph.node(pass_id).name
                    ))
                })?;
                let feed = mixed_composition_at(&composition, bed_id)?.clone();
                let gas = mixed_composition_at(&composition, pass_id)?.clone();
                let (products, pass, fluid) = bed_reactor(
                    graph,
                    slate,
                    enthalpy,
                    reactions,
                    edge_mass_flow,
                    edge_dissipation,
                    &temperature,
                    &composition,
                    &separations,
                    bed_id,
                    &feed,
                    catalyst,
                    wall,
                    Some(HydrogenInput {
                        pass: pass_id,
                        composition: &gas,
                        kinetics: *kinetics,
                        pressure,
                        consumption: consumption.as_ref(),
                    }),
                    dt,
                )?;
                composition.insert(bed_id, products);
                temperature.insert(bed_id, fluid);
                temperature.insert(pass_id, fluid);
                bed.insert(bed_id, pass);
            }
            [a, b] => {
                // Both outlets from one signed Q, computed from both inlets.
                let (t_a, t_b) = exchange_pair(
                    graph,
                    slate,
                    enthalpy,
                    edge_mass_flow,
                    edge_dissipation,
                    &temperature,
                    &composition,
                    &separations,
                    &previous.temperature,
                    (*a, *b),
                )?;
                temperature.insert(*a, t_a);
                temperature.insert(*b, t_b);
            }
            _ => {
                for &id in sides {
                    // A starved tank mixes its own inventory in as one more
                    // inflow, and takes NO heat term here: heat into a tank stays
                    // on the tank's own balance, as it does for a wet one, and
                    // adding it to the stream as well would count it twice
                    // (docs/DESIGN.md §28 fork 4, row B35).
                    if let Some((tank, supply)) = starved_tank(graph, starved, id) {
                        let own = InflowEnthalpy {
                            enthalpy_rate: enthalpy
                                .enthalpy_flux(
                                    slate,
                                    &tank.composition,
                                    KgPerSec(supply),
                                    tank.temperature,
                                )?
                                .value(),
                            mass_rate: supply,
                            capacity_rate: supply
                                * enthalpy
                                    .spot_cp(slate, &tank.composition, tank.temperature)?
                                    .value(),
                        };
                        let passing = inflow_totals(
                            graph,
                            slate,
                            enthalpy,
                            edge_mass_flow,
                            edge_dissipation,
                            &temperature,
                            &composition,
                            &separations,
                            id,
                        )?;
                        // Latent heat arriving on a two-phase inflow joins the
                        // sum, and what passes is settled at the tank's own
                        // pressure, as at a junction: a dry tank is swept as the
                        // pass-through it is (M53, docs/DESIGN.md §58 fork 4).
                        // Passing it on as liquid superheat instead put "liquid"
                        // naphtha at 184 °C in the product line of M53's plant.
                        let passing = passing.map(|totals| {
                            match inflow_latent(graph, edge_mass_flow, &vapour, id) {
                                Some(latent) => InflowEnthalpy {
                                    enthalpy_rate: totals.enthalpy_rate + latent,
                                    ..totals
                                },
                                None => totals,
                            }
                        });
                        // Nothing arriving: what passes, if anything, is what the
                        // tank held, at exactly its own temperature — not a mix of
                        // one term, which is the same number to within rounding.
                        let passed = match passing {
                            Some(totals) => {
                                let liquid_equivalent = enthalpy.mix_temperature(
                                    slate,
                                    mixed_composition_at(&composition, id)?,
                                    InflowEnthalpy {
                                        enthalpy_rate: own.enthalpy_rate + totals.enthalpy_rate,
                                        mass_rate: own.mass_rate + totals.mass_rate,
                                        capacity_rate: own.capacity_rate + totals.capacity_rate,
                                    },
                                )?;
                                settle_node(
                                    graph,
                                    slate,
                                    (line_flash, thermo, enthalpy),
                                    node_pressure,
                                    &composition,
                                    &mut vapour,
                                    id,
                                    liquid_equivalent,
                                )?
                            }
                            None => tank.temperature,
                        };
                        temperature.insert(id, passed);
                        continue;
                    }
                    // A reactor diverges from the ordinary mixing point in BOTH
                    // fields: it reacts the feed just mixed above into products,
                    // and it IMPOSES `t_set` instead of mixing a temperature. Its
                    // two emergent duties are recorded for the energy gate.
                    let reactor_cfg = match &graph.node(id).kind {
                        NodeKind::Reactor { t_set, tau } => Some((*t_set, *tau)),
                        _ => None,
                    };
                    if let Some((t_set, tau)) = reactor_cfg {
                        let feed = composition
                            .get(&id)
                            .expect("this vertex's composition was mixed above")
                            .clone();
                        let (products, duty) = reactor_duty(
                            graph,
                            slate,
                            enthalpy,
                            edge_mass_flow,
                            edge_dissipation,
                            &temperature,
                            &composition,
                            &separations,
                            id,
                            &feed,
                            t_set,
                            tau,
                            reactions,
                        )?;
                        composition.insert(id, products);
                        temperature.insert(id, t_set);
                        reactor_duties.insert(id, duty);
                    } else if let NodeKind::BedReactor {
                        bed: catalyst,
                        wall,
                    } = &graph.node(id).kind
                    {
                        // A bed reacts the feed just mixed above at its own
                        // start-of-tick temperature and takes the heat into
                        // itself (M57.1, docs/DESIGN.md §62.1). Like a furnace
                        // it is never HELD: with no inflow its fluid sits at the
                        // bed's temperature, which is a state.
                        let feed = mixed_composition_at(&composition, id)?.clone();
                        let (products, pass, fluid) = bed_reactor(
                            graph,
                            slate,
                            enthalpy,
                            reactions,
                            edge_mass_flow,
                            edge_dissipation,
                            &temperature,
                            &composition,
                            &separations,
                            id,
                            &feed,
                            catalyst,
                            wall,
                            None,
                            dt,
                        )?;
                        composition.insert(id, products);
                        temperature.insert(id, fluid);
                        bed.insert(id, pass);
                    } else if let NodeKind::Furnace {
                        duty,
                        coil,
                        flame_temperature,
                        tubes,
                    } = &graph.node(id).kind
                    {
                        // A furnace heats its fluid through its coil (M34,
                        // docs/DESIGN.md §37), so it is never HELD: with no
                        // inflow the fluid in its tubes sits at the coil's own
                        // temperature, which is a state, not a placeholder.
                        let pass = furnace_coil(
                            graph,
                            slate,
                            enthalpy,
                            edge_mass_flow,
                            edge_dissipation,
                            &temperature,
                            &composition,
                            &separations,
                            &vapour,
                            id,
                            *duty,
                            coil,
                            *flame_temperature,
                            tubes,
                            dt,
                        )?;
                        // What leaves the coil is settled by the line flash at
                        // the furnace's own pressure, as at a junction: a stream
                        // that boils in the tubes leaves partly vapour, at the
                        // flash's temperature (M62.4, docs/DESIGN.md §67.4).
                        // `NoLineFlash` hands the outlet back untouched.
                        let fluid = if pass.flowing {
                            settle_node(
                                graph,
                                slate,
                                (line_flash, thermo, enthalpy),
                                node_pressure,
                                &composition,
                                &mut vapour,
                                id,
                                pass.fluid,
                            )?
                        } else {
                            pass.fluid
                        };
                        temperature.insert(id, fluid);
                        coil_temperature.insert(id, pass.coil);
                        flue_loss.insert(id, pass.flue);
                        tube_fire.insert(id, pass.fire);
                    } else {
                        let mixed = match &graph.node(id).kind {
                            // A water cooler's duty is a RESULT of its two inlets
                            // (M62.4b, docs/DESIGN.md §67.13); the stream then
                            // mixes on what the water took, as a cooler's does.
                            NodeKind::WaterCooler {
                                ua,
                                water_inlet,
                                water_flow,
                            } => match water_cooler_pass(
                                graph,
                                slate,
                                (line_flash, thermo, enthalpy),
                                edge_mass_flow,
                                edge_dissipation,
                                (&temperature, &composition, &separations, &vapour),
                                node_pressure,
                                id,
                                (*ua, *water_inlet, *water_flow),
                            )? {
                                Some((pass, liquid_equivalent)) => {
                                    water_cooler.insert(id, pass);
                                    Some(liquid_equivalent)
                                }
                                None => None,
                            },
                            _ => mix_inflows(
                                graph,
                                slate,
                                enthalpy,
                                edge_mass_flow,
                                edge_dissipation,
                                &temperature,
                                &composition,
                                &separations,
                                &vapour,
                                id,
                            )?,
                        };
                        // No inflow: indeterminate but inert (mass balance ⇒ no
                        // outflow either), so the node HOLDS the last value it
                        // resolved — a placeholder that keeps the field finite and
                        // reproducible instead of dividing 0/0, and that is
                        // recorded as held so no control loop reads it as a
                        // measurement (docs/DESIGN.md §23 fork 4).
                        let mixed = match mixed {
                            // The mix is the LIQUID-EQUIVALENT temperature, its
                            // inflow's latent heat included; the line flash
                            // settles it at the node's own pressure, boiling
                            // what that enthalpy boils there (M53,
                            // docs/DESIGN.md §58 fork 2). `NoLineFlash` hands
                            // the mix back untouched.
                            //
                            // A COLUMN is not settled (M62.4, docs/DESIGN.md
                            // §67.4): its separation takes the whole feed at its
                            // liquid-equivalent temperature — the latent heat
                            // the feed carries in its enthalpy — and boils it on
                            // the column's own stages, and each draw leaves as a
                            // liquid at its own stage. A vapour share recorded
                            // here would be handed to every draw.
                            Some(liquid_equivalent)
                                if matches!(graph.node(id).kind, NodeKind::Column { .. }) =>
                            {
                                liquid_equivalent
                            }
                            Some(liquid_equivalent) => settle_node(
                                graph,
                                slate,
                                (line_flash, thermo, enthalpy),
                                node_pressure,
                                &composition,
                                &mut vapour,
                                id,
                                liquid_equivalent,
                            )?,
                            None => {
                                held.insert(id);
                                previous.temperature.get(&id).copied().unwrap_or(T_AMBIENT)
                            }
                        };
                        temperature.insert(id, mixed);

                        // A column separates its feed once, HERE: after its own
                        // temperature is mixed (the pass is a function of it) and
                        // before the sweep releases anything downstream (which
                        // reads the result through `edge_composition_at`). The
                        // column's own two fields are the ordinary mix — a column
                        // is not a T-overriding node the way a reactor is.
                        if let NodeKind::Column {
                            draws,
                            smearing,
                            pressure,
                            cascade,
                        } = &graph.node(id).kind
                        {
                            let feed = composition
                                .get(&id)
                                .expect("this vertex's composition was mixed above");
                            // The steam on its own inlet (M62.6, docs/DESIGN.md
                            // §67.18), resolved upstream before this column.
                            let steam = match cascade.as_ref().and_then(|spec| spec.steam) {
                                None => None,
                                Some(inlet) => Some(steam_into_column(
                                    graph,
                                    slate,
                                    enthalpy,
                                    edge_mass_flow,
                                    edge_dissipation,
                                    (&temperature, &composition, &separations, &vapour),
                                    id,
                                    inlet,
                                    mixed,
                                )?),
                            };
                            let pass = separation.separate(
                                &ColumnPass {
                                    slate,
                                    draws,
                                    smearing: *smearing,
                                    pressure: *pressure,
                                    feed,
                                    feed_flow: column_feed_flow(graph, edge_mass_flow, id),
                                    temperature: mixed,
                                    cascade: cascade.as_ref(),
                                    // The warm start, and it needs no engine
                                    // state: the previous tick's `NodeStates`
                                    // is already an argument to this sweep, and
                                    // its separations are already keyed by node.
                                    // A column that did not exist last tick, or
                                    // a fidelity that publishes no profile, is
                                    // `None` and starts cold.
                                    seed: previous
                                        .column_separation
                                        .get(&id)
                                        .and_then(|previous| previous.profile.as_ref()),
                                    steam,
                                },
                                thermo,
                            )?;
                            // Each draw LINE carries its draw on every pipe,
                            // `splitᵢ·ṁ_in` — the engine's draw write, to the
                            // bit (M62.4b, docs/DESIGN.md §67.13); `ṁ_in` the
                            // feed and any steam (M62.6).
                            let feed_flow = column_feed_flow(graph, edge_mass_flow, id).value();
                            let feed_flow = match steam {
                                None => feed_flow,
                                Some(steam) => feed_flow + steam.flow.value(),
                            };
                            for (column, index, line) in &lines {
                                if *column != id {
                                    continue;
                                }
                                let cut = pass.draws.get(*index).ok_or_else(|| {
                                    SimError::Numerical(format!(
                                        "separation model returned {} draws for column '{}'",
                                        pass.draws.len(),
                                        graph.node(id).name
                                    ))
                                })?;
                                let drawn = cut.split * feed_flow;
                                line_writes.extend(line.iter().map(|&edge| (edge, drawn)));
                            }
                            separations.insert(id, pass);
                        }
                    }
                }
            }
        }
        resolved += sides.len();

        // Release the downstream vertices this one feeds. An edge is an inflow
        // to the far node exactly when it is an outflow here: the two views
        // share one `flow`, so `into_other == -into_id` identically. Counting
        // per EDGE (not per node) mirrors `pending`, which counted inflow edges
        // — parallel pipes decrement once each, as they should.
        for &id in sides {
            for (edge, downstream, incoming) in graph.incident(id) {
                if downstream == id || !swept(downstream) {
                    continue;
                }
                let downstream_leader = leader_of(downstream);
                if downstream_leader == leader {
                    continue; // self-dependency: counted, never cleared (step 3)
                }
                // A draw line's pipe was counted by topology, and is cleared so
                // (M62.4b): out of `id` along the line, whatever its flow.
                if line_edge(edge) {
                    if incoming {
                        continue;
                    }
                } else {
                    let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
                    let into_id = if incoming { flow } else { -flow };
                    if into_id >= 0.0 {
                        continue; // not an outflow ⇒ not an inflow to `downstream`
                    }
                }
                if let Some(count) = pending.get_mut(&downstream_leader) {
                    *count = count.saturating_sub(1);
                    if *count == 0 {
                        ready.insert(downstream_leader);
                    }
                }
            }
        }
        if !line_writes.is_empty() {
            let written = flows.to_mut();
            for (edge, drawn) in line_writes {
                written.insert(edge, drawn);
            }
        }
    }

    // 5. A node left unresolved means its dependencies never cleared: the only
    //    way that happens is a cycle among zero-volume vertices — which now
    //    includes one side of an exchanger feeding the other, correctly, since
    //    that plant's two outlets really are mutually defined.
    if resolved < zero_volume.len() {
        let stuck: Vec<String> = zero_volume
            .iter()
            .filter(|id| !temperature.contains_key(id))
            .map(|id| graph.node(*id).name.clone())
            .collect();
        // A loop through a tank that has run dry has an inventory in it — just
        // none left to break the loop with. "Put a tank in the loop" is the wrong
        // advice there, so it gets its own message (docs/DESIGN.md §28 fork 4,
        // row B34).
        let dry: Vec<String> = zero_volume
            .iter()
            .filter(|id| !temperature.contains_key(id) && starved.contains_key(id))
            .map(|id| graph.node(*id).name.clone())
            .collect();
        if !dry.is_empty() {
            return Err(SimError::Numerical(format!(
                "recycle through a tank that has run dry ({}): the loop ({}) ran through \
                 an inventory while there was one, and the tank has no liquid left to break \
                 it this tick, so what it passes is its own outflow — a simultaneous solve \
                 of the loop's mix, which the upwind sweep does not do. A pump's \
                 minimum-flow line back to its own suction tank is the usual shape; keep \
                 that tank from running dry, or break the loop at another holdup \
                 (docs/DESIGN.md §28, DEFERRED B34).",
                dry.join(", "),
                stuck.join(", ")
            )));
        }
        return Err(SimError::Numerical(format!(
            "recycle through zero-volume nodes only ({}) — their temperatures are \
             mutually dependent with no inertial node to break the loop, which needs \
             a simultaneous solve the M2 upwind sweep does not implement. Put a tank \
             in the loop.",
            stuck.join(", ")
        )));
    }

    Ok(NodeStates {
        temperature,
        composition,
        reactor_duty: reactor_duties,
        column_separation: separations,
        held,
        coil_temperature,
        flue_loss,
        tube_fire,
        bed,
        vapour,
        tank_draw,
        water_cooler,
    })
}

/// Resolve a reactor: react its feed into products and compute its two emergent
/// duties. The caller stores `products` as the reactor's composition, `t_set` as
/// its temperature, and the returned `ReactorDuty` in `NodeStates`.
///
/// The emergent sensible duty is built from the SAME resolved inflows the
/// temperature mix would use — `inflow_totals` gives `Σ ṁ_in·cp_in·(T_in − T_REF)`
/// — plus the total inflow mass `ṁ` for the product-side term
/// `ṁ·cp_out·(T_set − T_REF)`, with `cp_out` taken from the PRODUCTS so the cp
/// shift is captured. The reactor is total-mass-neutral (`ṁ_out = ṁ_in`), so one
/// mass suffices for both ends.
///
/// A reactor with no inflow contributes zero of everything: `react` still runs
/// (on the fallback feed) so the composition field is defined, but with `ṁ = 0`
/// both duties are zero and there is no capacity to divide by.
#[allow(clippy::too_many_arguments)]
fn reactor_duty(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy_model: &dyn EnthalpyModel,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    edge_dissipation: &BTreeMap<EdgeId, Watt>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    separations: &BTreeMap<NodeId, Separation>,
    node: NodeId,
    feed: &Composition,
    t_set: Kelvin,
    tau: crate::units::Seconds,
    reactions: &dyn ReactionModel,
) -> Result<(Composition, ReactorDuty), SimError> {
    let reaction = reactions.react(feed, t_set, tau, slate)?;

    // Total inflow mass ṁ [kg/s]. `validate_degrees` pins the reactor at one
    // inflow, but summing is both correct and robust to that guard changing.
    let mass_in: f64 = inflow_edges(graph, edge_mass_flow, node)
        .iter()
        .map(|(_, _, into_node)| *into_node)
        .sum();

    // Σ ṁ_in·cp_in·(T_in − T_REF) [W] — the inlet enthalpy above the datum, the
    // same sum `mix_inflows` divides to get a mixed temperature. `None` = no
    // inflow, so no duty.
    let sensible = match inflow_totals(
        graph,
        slate,
        enthalpy_model,
        edge_mass_flow,
        edge_dissipation,
        temperature,
        composition,
        separations,
        node,
    )? {
        Some(totals) => {
            // The PRODUCTS' enthalpy at the setpoint, through the same model the
            // inlet sum came from — so the cp shift a conversion causes is booked
            // on one datum at both ends.
            let outlet_enthalpy = enthalpy_model
                .enthalpy_flux(slate, &reaction.products, KgPerSec(mass_in), t_set)?
                .value();
            outlet_enthalpy - totals.enthalpy_rate
        }
        None => 0.0,
    };
    let reported = sensible + mass_in * reaction.dh_rxn.value();

    Ok((
        reaction.products,
        ReactorDuty {
            sensible: Watt(sensible),
            reported: Watt(reported),
        },
    ))
}

/// Mass-weighted mix of a zero-volume node's inflow compositions.
///
/// ```text
/// f_c = Σ(ṁ_in · f_in,c) / Σ ṁ_in
/// ```
///
/// **The weight is `ṁ`, not `ṁ·cp`** — and that distinction is the one thing
/// this function exists to get right. `mix_inflows` sits ten lines away
/// weighting by `ṁ·cp`, because enthalpy is what mixes there; here the
/// conserved quantity is the mass of each component, and cp has nothing to do
/// with it. Reusing the enthalpy weights is the natural mistake and is nearly
/// invisible: it still conserves TOTAL mass and still passes every energy
/// balance, corrupting only the fractions — so it is caught by the
/// per-component invariant (I7) and the blend reference and by nothing else.
///
/// The two mixes stay consistent for free, which is why composition can ride
/// this sweep without perturbing M2's energy math: cp is linear in composition,
/// so `Σ_c (Σ_s ṁ_s·f_sc)·cp_c = Σ_s ṁ_s·cp_s` — the mixed stream's capacity
/// rate is exactly the sum of its inflows'.
///
/// # Errors
/// `SimError::Numerical` if an upwind composition is unresolved, or if the
/// accumulated weights are not a valid composition.
fn mix_compositions(
    graph: &PlantGraph,
    slate: &Slate,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    separations: &BTreeMap<NodeId, Separation>,
    composition: &BTreeMap<NodeId, Composition>,
    previous: &NodeStates,
    node: NodeId,
) -> Result<Composition, SimError> {
    // Accumulated in `inflow_edges` order, which is edge-id order — so the
    // float summation order is a function of the graph alone (rule 3).
    let mut weights = vec![0.0; slate.len()];
    let mut total_inflow = 0.0;

    for (edge, _upstream, into_node) in mixed_inflow_edges(graph, edge_mass_flow, node) {
        // The raw stored flow, not `into_node`: `upwind_end` picks the end from
        // the SIGN, and `inflow_edges` has already re-signed `into_node`
        // positive-into-this-node, which would name the wrong end on every edge
        // stored pointing inward. Same trap as the temperature path.
        let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
        let incoming = edge_composition_at(graph, separations, composition, edge, flow)?;
        for (weight, fraction) in weights.iter_mut().zip(incoming.fractions()) {
            *weight += into_node * fraction;
        }
        total_inflow += into_node;
    }

    if total_inflow > 0.0 {
        Composition::from_weights(&weights).map_err(|e| {
            SimError::Numerical(format!(
                "mixing the inflows of '{}' produced no valid composition: {e}",
                graph.node(node).name
            ))
        })
    } else {
        // No inflow: indeterminate but inert, mirroring `mix_inflows`' rule for
        // the same case exactly — mass balance forces zero outflow too, so
        // whatever is held here is carried nowhere. Holding the last resolved
        // value keeps the field reproducible; `Composition::from_weights` errs
        // on an all-zero weight vector, so mixing through it is not an option.
        //
        // With no history at all, what the node's first pipe carries — the
        // loader seeds every pipe with a component of its part of the plant's
        // phase — and the first slate component only on a node with no pipe.
        // The composition counterpart of the temperature path's fallback to
        // ambient. Not inert on a slate of two phases (M59.1, docs/DESIGN.md
        // §64): this is what the node's outlet pipe carries next, and a gas
        // line's density is read off it, so the first component — a liquid
        // lump on a hydrocracker's slate — put a closed vent's line at an oil's
        // density on the tick it opened. On a one-phase slate both answers are
        // the first component. A pipe whose composition does not span the
        // slate (a graph built by hand) is passed over.
        Ok(previous.composition.get(&node).cloned().unwrap_or_else(|| {
            graph
                .incident(node)
                .first()
                .map(|&(edge, _, _)| graph.pipe(edge).stream.composition.clone())
                .filter(|seed| seed.fractions().len() == slate.len())
                .unwrap_or_else(|| Composition::pure(slate.len(), 0))
        }))
    }
}

/// The tank and its supply [kg/s] if `node` is a tank the solve starved, else
/// `None` — the one test the sweep's two starved-tank branches share.
fn starved_tank<'a>(
    graph: &'a PlantGraph,
    starved: &BTreeMap<NodeId, StarvedTank>,
    node: NodeId,
) -> Option<(&'a crate::graph::TankState, f64)> {
    match (&graph.node(node).kind, starved.get(&node)) {
        (NodeKind::Tank(tank), Some(report)) => Some((tank, report.supply.value())),
        _ => None,
    }
}

/// A starved tank's pass-through composition: its own inventory, at the supply
/// rate, mixed by mass with whatever the network feeds it (docs/DESIGN.md §28
/// fork 4). `mix_compositions`' rule with one more inflow, and the same weights
/// — mass, not `ṁ·cp`.
///
/// With nothing arriving it is the tank's own composition exactly, rather than
/// a one-term mix of it, which is the same fractions to within a renormalising
/// rounding.
fn starved_tank_composition(
    graph: &PlantGraph,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    separations: &BTreeMap<NodeId, Separation>,
    composition: &BTreeMap<NodeId, Composition>,
    node: NodeId,
    own: &Composition,
    supply: f64,
) -> Result<Composition, SimError> {
    let inflows = inflow_edges(graph, edge_mass_flow, node);
    if inflows.is_empty() {
        return Ok(own.clone());
    }
    let mut weights: Vec<f64> = own.fractions().iter().map(|f| supply * f).collect();
    for (edge, _upstream, into_node) in inflows {
        // The raw stored flow, for the upwind pick; see `mix_compositions`.
        let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
        let incoming = edge_composition_at(graph, separations, composition, edge, flow)?;
        for (weight, fraction) in weights.iter_mut().zip(incoming.fractions()) {
            *weight += into_node * fraction;
        }
    }
    Composition::from_weights(&weights).map_err(|e| {
        SimError::Numerical(format!(
            "mixing what dry tank '{}' held with what it was fed produced no valid \
             composition: {e}",
            graph.node(node).name
        ))
    })
}

/// A node's inflow enthalpy [W] and capacity rate [W/K], or `None` when nothing
/// flows in.
///
/// The single definition of both sums. `mix_inflows` divides them to get a
/// mixed temperature; the exchanger needs the capacity rate itself, to size
/// `C_min`. Computing them in one place keeps the inlet temperature the
/// exchanger transfers heat *from* identical to the one an uncoupled node would
/// have mixed to.
type InflowTotals = Option<InflowEnthalpy>;

#[allow(clippy::too_many_arguments)]
fn inflow_totals(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy_model: &dyn EnthalpyModel,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    edge_dissipation: &BTreeMap<EdgeId, Watt>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    separations: &BTreeMap<NodeId, Separation>,
    node: NodeId,
) -> Result<InflowTotals, SimError> {
    let mut enthalpy = 0.0; // Σ ṁ·h(T) [W]
    let mut capacity = 0.0; // Σ ṁ·cp [W/K]
    let mut mass_rate = 0.0; // Σ ṁ [kg/s]

    for (edge, _upstream, into_node) in mixed_inflow_edges(graph, edge_mass_flow, node) {
        // The transformed OUTLET, not the raw upwind node temperature: with a
        // nonzero pipe `UA` those differ, and mixing the raw value would apply
        // the transform after the mixing that consumes it — i.e. never.
        //
        // `edge_mass_flow`, not `into_node`: the helper picks the upwind end
        // from the flow's SIGN, and `into_node` has already been re-signed
        // positive-into-this-node by `inflow_edges`. Feeding it that would name
        // the wrong end on every edge whose stored direction runs into `node`.
        // (This is the same class of error as passing a signed `ṁ` to the
        // transform, and it is why the helper takes the raw flow.)
        //
        // Resolution is guaranteed by construction — the sweep visits a node
        // only once every zero-volume upstream is done, and inertial ones are
        // seeded — but the helper returns an error rather than indexing, so a
        // sort bug can never panic (rule 5).
        let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
        let inlet_t = edge_temperature_at(
            graph,
            slate,
            enthalpy_model,
            temperature,
            composition,
            separations,
            edge,
            flow,
            dissipation_on(edge_dissipation, edge),
            node,
        )?;
        let crossing = edge_composition_at(graph, separations, composition, edge, flow)?;
        enthalpy += enthalpy_model
            .enthalpy_flux(slate, &crossing, KgPerSec(into_node), inlet_t)?
            .value();
        // The same resolved-upwind composition the transform above used, at the
        // same inlet temperature: an inlet transformed at one heat capacity and
        // mixed at another would not conserve enthalpy across the pipe.
        capacity += into_node * enthalpy_model.spot_cp(slate, &crossing, inlet_t)?.value();
        mass_rate += into_node;
    }

    Ok((capacity > 0.0).then_some(InflowEnthalpy {
        enthalpy_rate: enthalpy,
        mass_rate,
        capacity_rate: capacity,
    }))
}

/// Enthalpy-weighted mix of a zero-volume node's inflows [K], or `None` if the
/// node has no inflow at all.
///
/// `None` rather than a fallback value, because the fallback is a placeholder and
/// the caller is the one site that must both supply it and record that it did
/// (`NodeStates::held`, docs/DESIGN.md §23 fork 4).
#[allow(clippy::too_many_arguments)]
fn mix_inflows(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy_model: &dyn EnthalpyModel,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    edge_dissipation: &BTreeMap<EdgeId, Watt>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    separations: &BTreeMap<NodeId, Separation>,
    vapour: &BTreeMap<NodeId, VapourShare>,
    node: NodeId,
) -> Result<Option<Kelvin>, SimError> {
    // Heat — external (a fire) and a cooler's duty alike — joins the same first
    // law: for a node with no accumulation, Σ ṁ·h_in + Q = Σ ṁ·h_out. Without
    // this term `Command::SetHeatInput` would be a silent no-op on every
    // junction (the damage model's "fire on a node" would do nothing at all).
    // A FURNACE does not come through here since M34: its heat goes into its
    // coil first (`furnace_coil`, docs/DESIGN.md §37).
    let heat_input = heat_load(graph.node(node)).value();

    if let Some(totals) = inflow_totals(
        graph,
        slate,
        enthalpy_model,
        edge_mass_flow,
        edge_dissipation,
        temperature,
        composition,
        separations,
        node,
    )? {
        // The heat joins the ENTHALPY sum, not the temperature: `Q/ṁ` is a rise
        // in specific enthalpy whatever `cp` does, where `Q/(ṁ·cp)` is a rise in
        // temperature only while `cp` is flat.
        let heated = InflowEnthalpy {
            enthalpy_rate: totals.enthalpy_rate + heat_input,
            ..totals
        };
        // The latent heat a two-phase inflow carries joins the same sum (M53,
        // docs/DESIGN.md §58): the mix is then the temperature the node's inflow
        // would have as all LIQUID, which the line flash settles. Absent, not
        // zero, on a plant with no vapour, so this adds nothing to its bits.
        let heated = match inflow_latent(graph, edge_mass_flow, vapour, node) {
            Some(latent) => InflowEnthalpy {
                enthalpy_rate: heated.enthalpy_rate + latent,
                ..heated
            },
            None => heated,
        };
        let capacity = totals.capacity_rate;
        let mixed = enthalpy_model
            .mix_temperature(slate, mixed_composition_at(composition, node)?, heated)?
            .value();
        // A duty that exceeds the sensible heat available in the stream drives
        // the mix below absolute zero; `checked_temperature` owns that rule for
        // this path and the tank's alike (see its docs for why it Errs).
        checked_temperature(mixed, || {
            format!(
                "'{}' cools to {mixed:.2} K, below absolute zero: net heat load \
                 {heat_input:.4e} W exceeds the {capacity:.4e} W/K · {:.2} K of \
                 sensible heat its inflow carries above 0 K. Reduce the duty or \
                 raise the flow through it.",
                graph.node(node).name,
                totals.enthalpy_rate / capacity + T_REF.value(),
            )
        })
        .map(Some)
    } else {
        // No inflow: indeterminate but inert (mass balance ⇒ no outflow either),
        // and the caller supplies the held value.
        //
        // KNOWN LIMITATION: any `heat_input` here is dropped. A zero-volume node
        // has no thermal mass, so with no throughput there is nothing for the
        // heat to raise — the honest model of a fire against stagnant inventory
        // puts it on a Tank. Energy is therefore NOT conserved in this one case,
        // which is why `energy_invariants.rs` heats only tanks: it is a gap in
        // the model, not slack the invariant should be widened to tolerate. A
        // FURNACE is the one zero-volume unit that no longer has the gap: its
        // coil is the thermal mass (M34, docs/DESIGN.md §37, DEFERRED B39).
        Ok(None)
    }
}

/// One water cooler's tick (M62.4b, docs/DESIGN.md §67.13): its duty, its
/// cooling water's outlet, and the stream's liquid-equivalent temperature after
/// the water took the duty — which the caller settles by the line flash, as at
/// any cooler. `None` with no inflow: the caller holds the node, as a cooler's.
///
/// A counter-current exchanger between the stream and a cooling-water utility
/// (`COOLING_WATER_CP`), rated two ways:
///
/// - **The stream arrives liquid** (its inflow, settled at the cooler's
///   pressure, holds no vapour — it then only cools, so it stays liquid): the
///   counter-flow effectiveness, `Q = ε·C_min·(T_h,in − T_c,in)`,
///   `ε = (1 − e^{−NTU(1−C_r)})/(1 − C_r·e^{−NTU(1−C_r)})`, `NTU/(1 + NTU)` at
///   `C_r = 1`, `NTU = UA/C_min` (Incropera & DeWitt, 6th ed., Table 11.3), the
///   stream's capacity rate its inflow's spot one — closed form. Also where the
///   water is the warmer inlet, which heats the stream.
/// - **It arrives boiling** and condenses in the cooler: zone analysis (Kern,
///   *Process Heat Transfer*, 1950, ch. 12), `zone_duty`.
#[allow(clippy::too_many_arguments)] // one argument per seam the rating reads
#[allow(clippy::type_complexity)]
fn water_cooler_pass(
    graph: &PlantGraph,
    slate: &Slate,
    (line_flash, thermo, enthalpy): (&dyn LineFlashModel, &dyn ThermoModel, &dyn EnthalpyModel),
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    edge_dissipation: &BTreeMap<EdgeId, Watt>,
    (temperature, composition, separations, vapour): (
        &BTreeMap<NodeId, Kelvin>,
        &BTreeMap<NodeId, Composition>,
        &BTreeMap<NodeId, Separation>,
        &BTreeMap<NodeId, VapourShare>,
    ),
    node_pressure: &BTreeMap<NodeId, Pascal>,
    node: NodeId,
    (ua, water_inlet, water_flow): (WattPerKelvin, Kelvin, KgPerSec),
) -> Result<Option<(WaterCoolerPass, Kelvin)>, SimError> {
    let Some(totals) = inflow_totals(
        graph,
        slate,
        enthalpy,
        edge_mass_flow,
        edge_dissipation,
        temperature,
        composition,
        separations,
        node,
    )?
    else {
        return Ok(None);
    };
    let name = &graph.node(node).name;
    // What arrives [W]: the inflow's enthalpy, the latent heat a boiling inflow
    // carries, and a fire on the cooler, which heats the stream before the
    // water sees it.
    let arriving = totals.enthalpy_rate
        + inflow_latent(graph, edge_mass_flow, vapour, node).unwrap_or(0.0)
        + heat_load(graph.node(node)).value();
    let mix = mixed_composition_at(composition, node)?;
    let liquid_equivalent = |energy: f64| {
        enthalpy.mix_temperature(
            slate,
            mix,
            InflowEnthalpy {
                enthalpy_rate: energy,
                ..totals
            },
        )
    };
    // The stream's state at an enthalpy [W]: its liquid-equivalent, settled at
    // the cooler's pressure by the line flash (`NoLineFlash` hands it back).
    let state = |energy: f64| -> Result<Settled, SimError> {
        let le = liquid_equivalent(energy)?;
        if !line_flash.carries_vapour() {
            return Ok(Settled {
                temperature: le,
                vapour: None,
            });
        }
        let pressure = node_pressure.get(&node).copied().ok_or_else(|| {
            SimError::Numerical(format!(
                "internal: water cooler '{name}' has no solved pressure for its line flash"
            ))
        })?;
        line_flash.settle(slate, mix, le, pressure, thermo, enthalpy)
    };
    let inlet = state(arriving)?;
    let hot_in = inlet.temperature.value();
    let cold_in = water_inlet.value();
    let water_capacity = water_flow.value() * COOLING_WATER_CP;
    let duty = if water_capacity <= 0.0 {
        0.0
    } else if inlet.vapour.is_none() || hot_in <= cold_in {
        // Counter-flow effectiveness (Incropera & DeWitt, Table 11.3).
        let (c_min, c_max) = if totals.capacity_rate <= water_capacity {
            (totals.capacity_rate, water_capacity)
        } else {
            (water_capacity, totals.capacity_rate)
        };
        let c_r = c_min / c_max;
        let ntu = ua.value() / c_min;
        let effectiveness = if (1.0 - c_r).abs() < 1e-12 {
            ntu / (1.0 + ntu)
        } else {
            let decay = (-ntu * (1.0 - c_r)).exp();
            (1.0 - decay) / (1.0 - c_r * decay)
        };
        effectiveness * c_min * (hot_in - cold_in)
    } else {
        // The most the water can take: the stream cooled, all liquid, to the
        // water's inlet, or the water heated to the stream's inlet.
        let floor = enthalpy
            .enthalpy_flux(slate, mix, KgPerSec(totals.mass_rate), water_inlet)?
            .value();
        let ceiling = (arriving - floor).min(water_capacity * (hot_in - cold_in));
        zone_duty(
            name,
            arriving,
            hot_in,
            ceiling,
            (ua.value(), water_capacity, cold_in),
            |energy| state(energy).map(|settled| settled.temperature.value()),
        )?
    };
    let water_outlet = if water_capacity > 0.0 {
        cold_in + duty / water_capacity
    } else {
        cold_in
    };
    if water_outlet >= COOLING_WATER_BOILING.value() {
        return Err(SimError::Numerical(format!(
            "water cooler '{name}' would boil its cooling water: it takes {:.4} MW into \
             {:.4} kg/s of water from {:.2} °C, which would leave at {:.2} °C. Cooling \
             water is a liquid utility (docs/DESIGN.md §67.13): raise its flow \
             (`SetCoolingWaterFlow`) or cool the stream before it",
            duty / 1e6,
            water_flow.value(),
            cold_in - 273.15,
            water_outlet - 273.15
        )));
    }
    let outlet = liquid_equivalent(arriving - duty)?.value();
    let outlet = checked_temperature(outlet, || {
        format!("water cooler '{name}' cools its stream to {outlet:.2} K, below absolute zero")
    })?;
    Ok(Some((
        WaterCoolerPass {
            duty: Watt(duty),
            water_outlet: Kelvin(water_outlet),
        },
        outlet,
    )))
}

/// The duty [W] of a counter-current water cooler whose stream condenses in it
/// — zone analysis (Kern, *Process Heat Transfer*, 1950, ch. 12; M62.4b,
/// docs/DESIGN.md §67.13).
///
/// For a trial duty `Q` the stream's enthalpy falls from `arriving` in
/// `WATER_COOLER_ZONES` equal steps, its temperature at each from the line
/// flash (`hot_at`, a function of the stream's enthalpy [W]), and the water's
/// rises linearly from its inlet at the stream's outlet end:
/// `T_c,j = T_c,in + Q·(K − j)/(K·C_w)`. Each zone's area is `ΔQ/ΔT_lm`, and
/// `UA(Q)` their sum, which rises with `Q`. **A zone whose approach is zero or
/// less needs infinite area**: a condensing curve can pinch inside the
/// exchanger before `ceiling`, so the duty is the root of `UA(Q) = UA` below
/// the first that pinches — bisection where an end is infinite, the Illinois
/// regula falsi otherwise (Dowell & Jarratt, *BIT* 11, 1971).
fn zone_duty(
    name: &str,
    arriving: f64,
    hot_in: f64,
    ceiling: f64,
    (ua, water_capacity, cold_in): (f64, f64, f64),
    hot_at: impl Fn(f64) -> Result<f64, SimError>,
) -> Result<f64, SimError> {
    if ceiling <= 0.0 || ua <= 0.0 {
        return Ok(0.0);
    }
    let zones = WATER_COOLER_ZONES;
    let area_needed = |duty: f64| -> Result<f64, SimError> {
        let step = duty / zones as f64;
        let mut hot = hot_in;
        let mut cold = cold_in + duty / water_capacity;
        let mut area = 0.0;
        for j in 1..=zones {
            let next_hot = hot_at(arriving - step * j as f64)?;
            let next_cold = cold_in + duty * (zones - j) as f64 / (zones as f64 * water_capacity);
            let (a, b) = (hot - cold, next_hot - next_cold);
            if a <= 0.0 || b <= 0.0 {
                return Ok(f64::INFINITY);
            }
            // Log-mean temperature difference, its limit where the two agree.
            let lmtd = if (a - b).abs() <= 1e-12 * a.max(b) {
                0.5 * (a + b)
            } else {
                (a - b) / (a / b).ln()
            };
            area += step / lmtd;
            hot = next_hot;
            cold = next_cold;
        }
        Ok(area)
    };
    let (mut low, mut high) = (0.0_f64, ceiling);
    let (mut f_low, mut f_high) = (-ua, area_needed(ceiling)? - ua);
    if f_high <= 0.0 {
        return Ok(ceiling);
    }
    let mut side = 0i8;
    for _ in 0..WATER_COOLER_MAX_EVALUATIONS {
        let trial = if f_high.is_finite() {
            let secant = (low * f_high - high * f_low) / (f_high - f_low);
            if secant > low && secant < high {
                secant
            } else {
                0.5 * (low + high)
            }
        } else {
            0.5 * (low + high)
        };
        if trial <= low || trial >= high {
            return Ok(low);
        }
        let f = area_needed(trial)? - ua;
        if f == 0.0 {
            return Ok(trial);
        }
        if f < 0.0 {
            low = trial;
            f_low = f;
            if side == -1 && f_high.is_finite() {
                f_high *= 0.5;
            }
            side = -1;
        } else {
            high = trial;
            f_high = f;
            if side == 1 {
                f_low *= 0.5;
            }
            side = 1;
        }
        if high - low <= 4.0 * f64::EPSILON * high {
            return Ok(low);
        }
    }
    Err(SimError::Numerical(format!(
        "water cooler '{name}': its zone duty did not close within \
         {WATER_COOLER_MAX_EVALUATIONS} evaluations, bracketed in [{low:.6e}, {high:.6e}] W"
    )))
}

/// What one tick does to a furnace: its fluid's temperature and its coil's.
struct CoilPass {
    /// The fluid's temperature [K]: the outlet when something flows — its
    /// LIQUID-EQUIVALENT, the latent heat its inflow carried in included, which
    /// the caller settles by the line flash (M62.4, docs/DESIGN.md §67.4) — the
    /// coil's own end-of-tick temperature when nothing does.
    fluid: Kelvin,
    /// Whether anything flowed through the coil this tick: only then is there
    /// a stream for the line flash to settle.
    flowing: bool,
    /// The coil's temperature at the END of the tick [K].
    coil: Kelvin,
    /// The tick's average stack loss [W] (M36, docs/DESIGN.md §40).
    flue: Watt,
    /// The burn-out fire [W] (M37, docs/DESIGN.md §42).
    fire: Watt,
}

/// Integrate a furnace's coil across one tick and heat its fluid with what the
/// coil gives up (M34, docs/DESIGN.md §37), the coil absorbing only the share of
/// the duty its flame allows (M36, §40).
///
/// The model is `FurnaceCoil`'s: the fluid sees a tube wall at uniform `T_c`,
/// so the heat it takes is `G·(T_c − T_in)` with
/// `G = W·(1 − exp(−UA/W))`, `W = Σ ṁ·cp` its inlet capacity rate
/// (constant-surface-temperature tube flow, Incropera & DeWitt 6th ed.,
/// eq. 8.42b), and the coil is a lumped capacitance taking the rest
/// (ibid. §5.1). The duty `Q` is what the burners release: a well-stirred
/// firebox whose gas leaves at the coil's temperature, with a flue capacity
/// rate `Q/(T_f − T_a)` proportional to the firing, loses `K_f·(T_c − T_a)` up
/// the stack, `K_f = Q/(T_f − T_a)` (§40 fork 1). A burn-out fire `L` (M37,
/// §42) — what leaks from burst tubes, times its heating value — is fuel too,
/// so it fires beside the duty: `K_f = (Q + L)/(T_f − T_a)`. A commanded fire
/// `F` is a fixed heat into the metal, outside the flame law:
///
/// ```text
/// C·dT_c/dt = Q + L + F − K_f·(T_c − T_a) − G·(T_c − T_in)
/// ```
///
/// With everything but `T_c` frozen across the tick that is linear in `T_c`,
/// and its exact solution is
///
/// ```text
/// T_c(dt) = T_c + r₀ · dt/C · φ(x),   x = (G + K_f)·dt/C,   φ(x) = (1 − e^−x)/x
/// ```
///
/// with `r₀` the right-hand side at the start of the tick — computed in that
/// form, with `φ(0) = 1`, because the textbook form `T_eq + (T_c − T_eq)·e^−x`
/// divides by `G + K_f`, and an unlit furnace with no flow has both at zero.
/// Exact means unconditionally stable: no coil is too light for `dt`, and none
/// is refused for it. With no flow the coil approaches `T_f` (plus `F/K_f`)
/// and never crosses it: the ceiling.
///
/// **The first law, four terms**: fired in, `C·ΔT_c` stored, the flue's
/// `K_f·(T̄_c − T_a)` at the tick's AVERAGE coil `T̄_c = T_c + r₀·dt/C·ψ(x)`
/// from the same exact solution, and the fluid takes what is left, added to the
/// inflow ENTHALPY sum as a furnace's duty used to be. A remainder rather than
/// its own formula, so an error in `ψ` moves heat between the flue and the
/// fluid but never creates it.
///
/// **The outlet never exceeds the coil's hotter end of the tick.** The fluid's
/// rise is `ε` times the tick's average `T_c − T_in`, with `ε ≤ 1`. It CAN sit
/// above the coil's end-of-tick value on a tick when the coil is cooling, which
/// is the value a snapshot shows (`FurnaceCoil::temperature`).
///
/// **No flow**: `G = 0`, so the coil keeps everything the flame lets it absorb,
/// and the fluid standing in the tubes is reported at the coil's end-of-tick
/// temperature. Nothing is dropped and nothing is held.
#[allow(clippy::too_many_arguments)]
fn furnace_coil(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy_model: &dyn EnthalpyModel,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    edge_dissipation: &BTreeMap<EdgeId, Watt>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    separations: &BTreeMap<NodeId, Separation>,
    vapour: &BTreeMap<NodeId, VapourShare>,
    node: NodeId,
    duty: Watt,
    coil: &FurnaceCoil,
    flame_temperature: Kelvin,
    tubes: &FurnaceTubes,
    dt: Seconds,
) -> Result<CoilPass, SimError> {
    let name = &graph.node(node).name;
    let leak_fire = burnout_fire(edge_mass_flow, name, tubes)?;
    // Duty, burn-out fire and any commanded fire all go into the metal:
    // `heat_load` is still the one owner of the duty and the commanded fire.
    // The duty and the burn-out fire are fuel, so they have a flue; a
    // commanded fire is not, and has none (§40 fork 2).
    let fired = heat_load(graph.node(node)).value() + leak_fire;
    let air = T_AMBIENT.value();
    // Flue conductance, K_f = Q/(T_f − T_a) (§40 fork 1), with Q the FUEL
    // fired: the duty plus the burn-out fire. Exactly zero unlit and intact.
    let flue_conductance = (duty.value() + leak_fire) / (flame_temperature.value() - air);
    let capacity = coil.heat_capacity.value();
    let ua = coil.conductance.value();
    let start = coil.temperature.value();

    let totals = inflow_totals(
        graph,
        slate,
        enthalpy_model,
        edge_mass_flow,
        edge_dissipation,
        temperature,
        composition,
        separations,
        node,
    )?;
    // The latent heat a two-phase inflow carries joins the inflow's enthalpy, as
    // at any mixing point (M62.4, docs/DESIGN.md §67.4): the inlet is then its
    // liquid-equivalent temperature. Absent, not zero, on a plant with no
    // vapour, so this adds nothing to its bits.
    let totals = totals.map(
        |totals| match inflow_latent(graph, edge_mass_flow, vapour, node) {
            Some(latent) => InflowEnthalpy {
                enthalpy_rate: totals.enthalpy_rate + latent,
                ..totals
            },
            None => totals,
        },
    );
    // The fluid's inlet temperature and the coil's conductance to it. With no
    // inflow there is no fluid to heat: `G = 0` and the inlet is never read.
    let (inlet, conductance) = match totals {
        Some(totals) => {
            let inlet = enthalpy_model
                .mix_temperature(slate, mixed_composition_at(composition, node)?, totals)?
                .value();
            // ε = 1 − exp(−NTU), NTU = UA/W (Incropera & DeWitt eq. 8.42b).
            // `-expm1(-ntu)` rather than `1 - exp(-ntu)`: exact at small NTU,
            // and exactly 1 when a vanishing flow sends NTU to infinity.
            let effectiveness = -(-ua / totals.capacity_rate).exp_m1();
            (inlet, totals.capacity_rate * effectiveness)
        }
        None => (start, 0.0),
    };

    // The exact step, in the form that never divides by a conductance (see the
    // docs). `rate` is the right-hand side at the start of the tick [W].
    let x = (conductance + flue_conductance) * dt.value() / capacity;
    let rate = fired - flue_conductance * (start - air) - conductance * (start - inlet);
    let phi = if x > 0.0 { -(-x).exp_m1() / x } else { 1.0 };
    let end = start + rate * dt.value() / capacity * phi;
    if !end.is_finite() {
        return Err(SimError::Numerical(format!(
            "furnace '{name}' coil temperature is not finite ({end}) after a tick fired \
             at {fired:.4e} W from {start:.2} K"
        )));
    }
    let coil_end = checked_temperature(end, || {
        format!(
            "furnace '{name}' coil cools to {end:.2} K, below absolute zero, from \
             {start:.2} K on a fired heat of {fired:.4e} W"
        )
    })?;

    // The stack loss at the tick's average coil (§40 fork 3). Zero unlit.
    let flue = if flue_conductance > 0.0 {
        let average = start + rate * dt.value() / capacity * psi(x);
        flue_conductance * (average - air)
    } else {
        0.0
    };

    let Some(totals) = totals else {
        return Ok(CoilPass {
            fluid: coil_end,
            flowing: false,
            coil: coil_end,
            flue: Watt(flue),
            fire: Watt(leak_fire),
        });
    };
    // What the coil gave the fluid this tick, as an average power [W]: fired
    // in, less what it now stores and what went up the stack. The first law at
    // the furnace, closed by construction.
    //
    // From the STORED change, `end − start` (exact: the two are close), not from
    // the increment before it was added: the bits of the increment that
    // rounding drops from `end` would otherwise reach the fluid without leaving
    // the coil. The price is on a vanishing flow, where one rounding step of the
    // coil, `C·ulp(T_c)/dt`, is spread over a capacity rate near zero and can
    // put the outlet that far above the coil.
    let to_fluid = fired - capacity * (end - start) / dt.value() - flue;
    let heated = InflowEnthalpy {
        enthalpy_rate: totals.enthalpy_rate + to_fluid,
        ..totals
    };
    let outlet = enthalpy_model
        .mix_temperature(slate, mixed_composition_at(composition, node)?, heated)?
        .value();
    let fluid = checked_temperature(outlet, || {
        format!(
            "furnace '{name}' outlet {outlet:.2} K is below absolute zero: its coil gave \
             {to_fluid:.4e} W to an inflow of {:.4e} W/K",
            totals.capacity_rate
        )
    })?;
    Ok(CoilPass {
        fluid,
        flowing: true,
        coil: coil_end,
        flue: Watt(flue),
        fire: Watt(leak_fire),
    })
}

/// The burn-out fire [W] a furnace's failed tubes feed this tick (M37,
/// docs/DESIGN.md §42), zero while they are intact.
fn burnout_fire(
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    name: &str,
    tubes: &FurnaceTubes,
) -> Result<f64, SimError> {
    // The burn-out fire (M37, docs/DESIGN.md §42): while the tubes are FAILED,
    // what flows out of their hole this tick burns in the firebox. It is FUEL —
    // the process fluid burning in the burners' air — so unlike a commanded
    // fire it fires through the flame law below, as extra firing beside the
    // duty. Read off THIS tick's solve, which never reads a duty, so nothing is
    // circular and nothing is stored. Only an OUTWARD flow burns; a hole the
    // plant draws in through is refused by the solve before it gets here.
    Ok(if tubes.state.is_failed() {
        let hole = tubes.hole.ok_or_else(|| {
            SimError::Scenario(format!(
                "furnace '{name}' has burst tubes but owns no burn-out hole. The loader \
                 builds one on every furnace's outlet pipe; a graph built by hand must do \
                 the same (docs/DESIGN.md §42)"
            ))
        })?;
        let leaked = edge_mass_flow.get(&hole).copied().ok_or_else(|| {
            SimError::Numerical(format!(
                "furnace '{name}': the solve reported no flow for its burn-out hole {hole:?}"
            ))
        })?;
        leaked.max(0.0) * tubes.heating_value.value()
    } else {
        0.0
    })
}

/// Iterations of a celled coil's fixed point (§67.14): its cells' inlets
/// move the metal's average and the average the inlets. The metal's heat
/// capacity is large against a tick's conductance, so a few suffice; past this
/// it is an `Err`, never a silent answer.
const COIL_CELL_MAX_PASSES: u32 = 50;

/// What the fixed point closes on [K]: the largest move of any cell's inlet
/// between two passes.
const COIL_CELL_TOLERANCE_K: f64 = 1e-9;

/// One tick of a furnace whose coil is cut into cells (M62.4c, docs/DESIGN.md
/// §67.14): `chain` its cells in the stream's order, the furnace last.
///
/// One metal at `T_c`, the lumped capacitance of `furnace_coil`; each of the
/// `N` cells holds `UA/N` and exchanges `G_i·(T̄_c − T_in,i)` with it,
/// `G_i = W_i·(1 − e^{−UA_i/W_i})` on its inflow's capacity rate (Incropera &
/// DeWitt eq. 8.42b), `T_in,i` its inflow settled at its own pressure — the
/// first cell's from outside, every other's the cell before it, through the
/// segment between them (its friction heat and the latent heat the vapour
/// carries included, `inflow_totals` and `inflow_latent`). The metal's exact
/// step runs on `G = ΣG_i` and `Σ G_i·(T_c − T_in,i)`; the fluid takes what the
/// first law leaves, each cell `G_i·(T̄_c − T_in,i)` and the last the
/// remainder, so the books close by construction. Each cell's outlet is
/// settled by the line flash at its own pressure (`settle_node`), recorded in
/// `vapour` for the next cell, the edge write and the next solve. The inlets
/// move the average and the average the inlets: iterated to
/// `COIL_CELL_TOLERANCE_K`.
#[allow(clippy::too_many_arguments)] // one argument per seam the coil reads
#[allow(clippy::type_complexity)]
fn furnace_coil_cells(
    graph: &PlantGraph,
    slate: &Slate,
    (line_flash, thermo, enthalpy): (&dyn LineFlashModel, &dyn ThermoModel, &dyn EnthalpyModel),
    (edge_mass_flow, edge_dissipation): (&BTreeMap<EdgeId, f64>, &BTreeMap<EdgeId, Watt>),
    (temperature, composition, separations, vapour): (
        &mut BTreeMap<NodeId, Kelvin>,
        &BTreeMap<NodeId, Composition>,
        &BTreeMap<NodeId, Separation>,
        &mut BTreeMap<NodeId, VapourShare>,
    ),
    node_pressure: &BTreeMap<NodeId, Pascal>,
    previous: &NodeStates,
    chain: &[NodeId],
    (duty, coil, flame_temperature, tubes): (Watt, &FurnaceCoil, Kelvin, &FurnaceTubes),
    dt: Seconds,
) -> Result<CoilPass, SimError> {
    let furnace = chain[chain.len() - 1];
    let name = &graph.node(furnace).name;
    let leak_fire = burnout_fire(edge_mass_flow, name, tubes)?;
    let fired = heat_load(graph.node(furnace)).value() + leak_fire;
    let air = T_AMBIENT.value();
    let flue_conductance = (duty.value() + leak_fire) / (flame_temperature.value() - air);
    let capacity = coil.heat_capacity.value();
    let cell_ua = coil.conductance.value() / chain.len() as f64;
    let start = coil.temperature.value();

    // A cell's inflow [W, kg/s, W/K], its latent heat included.
    let arriving = |temperature: &BTreeMap<NodeId, Kelvin>,
                    vapour: &BTreeMap<NodeId, VapourShare>,
                    cell: NodeId|
     -> Result<Option<InflowEnthalpy>, SimError> {
        let totals = inflow_totals(
            graph,
            slate,
            enthalpy,
            edge_mass_flow,
            edge_dissipation,
            temperature,
            composition,
            separations,
            cell,
        )?;
        Ok(totals.map(
            |totals| match inflow_latent(graph, edge_mass_flow, vapour, cell) {
                Some(latent) => InflowEnthalpy {
                    enthalpy_rate: totals.enthalpy_rate + latent,
                    ..totals
                },
                None => totals,
            },
        ))
    };
    let mix = mixed_composition_at(composition, chain[0])?;
    // A stream's temperature where it stands: its liquid-equivalent settled at
    // the cell's pressure (`NoLineFlash` hands it back).
    let standing = |le: Kelvin, cell: NodeId| -> Result<Kelvin, SimError> {
        if !line_flash.carries_vapour() {
            return Ok(le);
        }
        let pressure = node_pressure.get(&cell).copied().ok_or_else(|| {
            SimError::Numerical(format!(
                "internal: coil cell '{}' has no solved pressure for its line flash",
                graph.node(cell).name
            ))
        })?;
        Ok(line_flash
            .settle(slate, mix, le, pressure, thermo, enthalpy)?
            .temperature)
    };

    let Some(first) = arriving(temperature, vapour, chain[0])? else {
        // No flow: the metal keeps all the flame lets it absorb, and the fluid
        // standing in every cell is reported at the metal's end-of-tick
        // temperature, as in a lumped coil.
        let step = coil_step(fired, flue_conductance, capacity, start, 0.0, 0.0, dt, name)?;
        for &cell in chain {
            temperature.insert(cell, Kelvin(step.end));
        }
        return Ok(CoilPass {
            fluid: Kelvin(step.end),
            flowing: false,
            coil: Kelvin(step.end),
            flue: Watt(step.flue),
            fire: Watt(leak_fire),
        });
    };

    // The first guess of every inlet: the coil's own inflow where it stands,
    // and each later cell's last resolved temperature.
    let first_inlet = standing(enthalpy.mix_temperature(slate, mix, first)?, chain[0])?.value();
    let mut inlets: Vec<f64> = (0..chain.len())
        .map(|i| match i {
            0 => first_inlet,
            _ => previous
                .temperature
                .get(&chain[i - 1])
                .map_or(first_inlet, |t| t.value()),
        })
        .collect();
    let mut capacities = vec![first.capacity_rate; chain.len()];
    for _ in 0..COIL_CELL_MAX_PASSES {
        let conductances: Vec<f64> = capacities
            .iter()
            .map(|&w| w * -(-cell_ua / w).exp_m1())
            .collect();
        let conductance: f64 = conductances.iter().sum();
        let exchange: f64 = conductances
            .iter()
            .zip(&inlets)
            .map(|(g, t_in)| g * (start - t_in))
            .sum();
        let step = coil_step(
            fired,
            flue_conductance,
            capacity,
            start,
            conductance,
            exchange,
            dt,
            name,
        )?;
        let mut taken = 0.0;
        let mut moved = 0.0_f64;
        let mut outlet = Kelvin(step.end);
        for (i, &cell) in chain.iter().enumerate() {
            let totals = arriving(temperature, vapour, cell)?.ok_or_else(|| {
                SimError::Numerical(format!(
                    "internal: coil cell '{}' has no inflow while its coil does",
                    graph.node(cell).name
                ))
            })?;
            capacities[i] = totals.capacity_rate;
            let inlet = standing(enthalpy.mix_temperature(slate, mix, totals)?, cell)?.value();
            moved = moved.max((inlet - inlets[i]).abs());
            inlets[i] = inlet;
            let heat = if i + 1 < chain.len() {
                conductances[i] * (step.average - inlet)
            } else {
                step.to_fluid - taken
            };
            taken += heat;
            let le = enthalpy
                .mix_temperature(
                    slate,
                    mix,
                    InflowEnthalpy {
                        enthalpy_rate: totals.enthalpy_rate + heat,
                        ..totals
                    },
                )?
                .value();
            let le = checked_temperature(le, || {
                format!(
                    "coil cell '{}' of furnace '{name}' leaves at {le:.2} K, below absolute zero",
                    graph.node(cell).name
                )
            })?;
            vapour.remove(&cell);
            outlet = settle_node(
                graph,
                slate,
                (line_flash, thermo, enthalpy),
                node_pressure,
                composition,
                vapour,
                cell,
                le,
            )?;
            temperature.insert(cell, outlet);
        }
        if moved <= COIL_CELL_TOLERANCE_K {
            return Ok(CoilPass {
                fluid: outlet,
                flowing: true,
                coil: Kelvin(step.end),
                flue: Watt(step.flue),
                fire: Watt(leak_fire),
            });
        }
    }
    Err(SimError::Numerical(format!(
        "furnace '{name}': its coil's cells did not settle on one metal within \
         {COIL_CELL_MAX_PASSES} passes"
    )))
}

/// One tick of a coil's metal (§37, §40): its end temperature, its average
/// over the tick, the flue loss at that average, and what the fluid took.
struct CoilStep {
    end: f64,
    average: f64,
    flue: f64,
    to_fluid: f64,
}

/// The coil's exact step, `furnace_coil`'s arithmetic for `G = conductance`
/// and an exchange at the start of the tick of `exchange` [W] — `G·(T_c −
/// T_in)` on a lumped coil, `Σ G_i·(T_c − T_in,i)` on cells.
#[allow(clippy::too_many_arguments)] // the step's own inputs
fn coil_step(
    fired: f64,
    flue_conductance: f64,
    capacity: f64,
    start: f64,
    conductance: f64,
    exchange: f64,
    dt: Seconds,
    name: &str,
) -> Result<CoilStep, SimError> {
    let air = T_AMBIENT.value();
    let x = (conductance + flue_conductance) * dt.value() / capacity;
    let rate = fired - flue_conductance * (start - air) - exchange;
    let phi = if x > 0.0 { -(-x).exp_m1() / x } else { 1.0 };
    let end = start + rate * dt.value() / capacity * phi;
    if !end.is_finite() {
        return Err(SimError::Numerical(format!(
            "furnace '{name}' coil temperature is not finite ({end}) after a tick fired \
             at {fired:.4e} W from {start:.2} K"
        )));
    }
    let end = checked_temperature(end, || {
        format!(
            "furnace '{name}' coil cools to {end:.2} K, below absolute zero, from \
             {start:.2} K on a fired heat of {fired:.4e} W"
        )
    })?
    .value();
    let average = start + rate * dt.value() / capacity * psi(x);
    let flue = if flue_conductance > 0.0 {
        flue_conductance * (average - air)
    } else {
        0.0
    };
    let to_fluid = fired - capacity * (end - start) / dt.value() - flue;
    Ok(CoilStep {
        end,
        average,
        flue,
        to_fluid,
    })
}

/// `ψ(x) = (1 − φ(x))/x = (x − 1 + e^−x)/x²`, `ψ(0) = ½`: the weight that turns
/// a linear relaxation's initial rate into its average displacement over a
/// step (§40 fork 3). The series below `1e-3`, where the closed form cancels.
fn psi(x: f64) -> f64 {
    if x < 1.0e-3 {
        // ψ(x) = ½ − x/6 + x²/24 − x³/120 + …; the next term is x⁴/720.
        0.5 - x / 6.0 + x * x / 24.0 - x * x * x / 120.0
    } else {
        (x + (-x).exp_m1()) / (x * x)
    }
}

/// Integrate a catalyst bed across one tick: react its feed at the bed's
/// START-of-tick temperature, put the released heat into the bed, and send the
/// fluid out at the bed's temperature (M57.1, docs/DESIGN.md §62.1).
///
/// Returns the products (the bed's composition), what the tick did to the bed,
/// and the fluid's temperature.
///
/// **The residence time follows the flow** (§62.2): `τ = V·ρ_feed/ṁ`, the space
/// time `1/LHSV` fixed-bed kinetics are quoted against, with `ρ_feed` the feed's
/// liquid density (`Composition::mixture_density`, the mass-weighted harmonic
/// mean of the slate's declared densities). At zero flow `τ = +∞`.
///
/// **The bed's first law**, the lumped capacitance of `furnace_coil` with the
/// flame replaced by the chemistry (Incropera & DeWitt, *Fundamentals of Heat
/// and Mass Transfer*, 6th ed., §5.1):
///
/// ```text
/// C·dT_b/dt = Q + F + H_in − H_out(T_b),   Q = −ṁ·Δh_rxn(T_b, τ)
/// ```
///
/// with `F` the fire on the bed — a commanded fire (`Command::SetHeatInput`), a
/// fixed heat into it as on a coil, plus, while its wall is failed, the share
/// `s·L` of the fire its leak feeds (M58, docs/DESIGN.md §63.3) — and
/// `H_out` the PRODUCTS' enthalpy rate at the bed's temperature, so the cp
/// shift a conversion causes is booked (§5 fork 2).
/// `Q`, `F` and `H_in` are frozen across the tick and `H_out` is linearised by
/// the products' capacity rate `W = ṁ·cp_out(T_b)`, so its exact solution is
/// `furnace_coil`'s:
///
/// ```text
/// T_b(dt) = T_b + r₀·dt/C·φ(x),   x = W·dt/C,   φ(x) = (1 − e^−x)/x
/// ```
///
/// `r₀` the right-hand side at the start of the tick. Exact for the frozen rate
/// and unconditionally stable, and exactly `(Q + F)·dt/C` at zero flow.
///
/// **The fluid's heat comes from the STORED change**, as the coil's does
/// (§37 fork 3, correction 2): `H_out,avg = H_in + Q + F − C·(T_b,end −
/// T_b,start)/dt`, turned into a temperature on the products' own enthalpy, so
/// the first law at the bed closes by construction. Fluid–solid contact is
/// complete (`ε = 1`, §62.1): the outlet is the tick's average bed temperature
/// under the linearised outflow.
///
/// **No flow**: `ṁ = 0`, so nothing reacts into or out of the bed (`Q = 0`)
/// and nothing carries heat away; the bed keeps its temperature, plus any fire.
/// The fluid standing in it is reported at the bed's end-of-tick temperature
/// with the products of complete conversion (`τ = +∞`). Nothing is held.
/// A catalyst bed's hydrogen, as the paired sweep vertex hands it to
/// `bed_reactor` (M59.1, docs/DESIGN.md §64.2): the pass node, the hydrogen it
/// mixed, the bed's kinetics, and the pressure the pass solved to this tick.
struct HydrogenInput<'a> {
    pass: NodeId,
    composition: &'a Composition,
    kinetics: HydrogenKinetics,
    pressure: Pascal,
    /// The pass's consumption (M59.2), `None` on a pass that declares none.
    consumption: Option<&'a HydrogenConsumption>,
}

#[allow(clippy::too_many_arguments)]
fn bed_reactor(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy_model: &dyn EnthalpyModel,
    reactions: &dyn ReactionModel,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    edge_dissipation: &BTreeMap<EdgeId, Watt>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    separations: &BTreeMap<NodeId, Separation>,
    node: NodeId,
    feed: &Composition,
    bed: &CatalystBed,
    wall: &BedWall,
    hydrogen: Option<HydrogenInput<'_>>,
    dt: Seconds,
) -> Result<(Composition, BedPass, Kelvin), SimError> {
    let name = &graph.node(node).name;
    let capacity = bed.heat_capacity.value();
    let start = bed.temperature.value();
    // The wall fire (M58, docs/DESIGN.md §63.3): while the wall is FAILED, what
    // flows out of its hole this tick burns outside the vessel, `L = ṁ·LHV`,
    // read off THIS tick's solve — §42 fork 3's fire, nothing circular and
    // nothing stored. Only an OUTWARD flow burns. It is read whether or not
    // the bed itself has flow: a hole on its outlet can be fed from downstream.
    // A bed whose pass leaks hydrogen burns both holes, each at its own heating
    // value (M59.4, docs/DESIGN.md §64.7).
    let wall_fire = if wall.state.is_failed() {
        let burning = std::iter::once((wall.hole, wall.heating_value)).chain(
            wall.hydrogen_leak
                .as_ref()
                .map(|leak| (leak.hole, leak.heating_value)),
        );
        let mut fire = 0.0;
        for (hole, heating_value) in burning {
            let hole = hole.ok_or_else(|| {
                SimError::Scenario(format!(
                    "catalyst bed '{name}' has a failed wall but owns no hole. The loader builds \
                     one on every bed's outlet pipe, and on its hydrogen pass's when the pass \
                     declares a heating value; a graph built by hand must do the same \
                     (docs/DESIGN.md §63.1, §64.7)"
                ))
            })?;
            let leaked = edge_mass_flow.get(&hole).copied().ok_or_else(|| {
                SimError::Numerical(format!(
                    "catalyst bed '{name}': the solve reported no flow for its wall's hole \
                     {hole:?}"
                ))
            })?;
            fire += leaked.max(0.0) * heating_value.value();
        }
        fire
    } else {
        0.0
    };
    // What heats the bed besides its chemistry: a commanded fire, as on a
    // furnace's coil — a bed has the thermal mass a fire on a zero-volume node
    // lacks (`mix_inflows`) — and the wall fire's declared share, a game value
    // (§63.3); the rest of the wall fire goes to the air.
    let fire = heat_load(graph.node(node)).value() + wall.fire_share * wall_fire;
    let wall_fire = Watt(wall_fire);

    let totals = inflow_totals(
        graph,
        slate,
        enthalpy_model,
        edge_mass_flow,
        edge_dissipation,
        temperature,
        composition,
        separations,
        node,
    )?;

    // The hydrogen (M59.1, docs/DESIGN.md §64): the pass's inflow, and the
    // factor `f = (p/p_ref)^n` its pressure sets.
    let hydrogen = match hydrogen {
        Some(input) => {
            let factor = input.kinetics.factor(input.pressure);
            if !factor.is_finite() {
                return Err(SimError::Numerical(format!(
                    "catalyst bed '{name}': its hydrogen factor at {:.4e} Pa is {factor}",
                    input.pressure.value()
                )));
            }
            let inflow = inflow_totals(
                graph,
                slate,
                enthalpy_model,
                edge_mass_flow,
                edge_dissipation,
                temperature,
                composition,
                separations,
                input.pass,
            )?;
            // What the gas side lost at the pass (M59.2, docs/DESIGN.md
            // §64.11): its net inflow in this solve, read only where the pass
            // consumes — elsewhere it is the solve's residual and nothing.
            let consumed = input
                .consumption
                .map(|_| net_inflow(graph, edge_mass_flow, input.pass));
            Some((input, factor, inflow, consumed))
        }
        None => None,
    };

    // τ = V·ρ_feed/ṁ (§62.2). The density is the liquid one, and the loader
    // refuses a bed in gas service; this refusal is the engine's own guard
    // (rule 5), since `mixture_density` has none in release.
    let residence_time = match totals {
        Some(totals) => {
            if feed.phase(slate)? != Phase::Liquid {
                return Err(SimError::Numerical(format!(
                    "catalyst bed '{name}' is fed a gas: its residence time is the feed's \
                     LIQUID volume over the catalyst's (docs/DESIGN.md §62.2)"
                )));
            }
            let density = feed.mixture_density(slate).value();
            Some(Seconds(
                bed.catalyst_volume.value() * density / totals.mass_rate,
            ))
        }
        None => None,
    };
    let space_time = residence_time.unwrap_or(Seconds(f64::INFINITY));
    // Every rate times `f` is the same chemistry over the space time `f·τ`: the
    // network is first order, `dy/dτ = f·K·y` (§64.1). At `f = 0` nothing
    // reacts, whatever `τ` — written out, since `0·∞` is NaN.
    let space_time = match &hydrogen {
        Some((_, factor, _, _)) if *factor > 0.0 => Seconds(space_time.value() * factor),
        Some(_) => Seconds(0.0),
        None => space_time,
    };
    let reaction = reactions.react(feed, bed.temperature, space_time, slate)?;
    if !reaction.dh_rxn.value().is_finite() {
        return Err(SimError::Numerical(format!(
            "catalyst bed '{name}': the reaction model '{}' returned a heat of reaction of \
             {} J/kg at {start:.2} K",
            reactions.name(),
            reaction.dh_rxn.value()
        )));
    }

    if let Some(hydrogen) = hydrogen {
        return bed_with_hydrogen(
            name,
            (slate, enthalpy_model),
            bed,
            (totals, feed),
            reaction,
            residence_time,
            hydrogen,
            (fire, wall_fire),
            dt,
        );
    }

    let Some(totals) = totals else {
        // No flow: the fire alone, exactly.
        let end = start + fire * dt.value() / capacity;
        let end = checked_bed_temperature(name, end, start, fire)?;
        return Ok((
            reaction.products,
            BedPass {
                temperature: end,
                reaction_heat: Watt::ZERO,
                residence_time: None,
                wall_fire,
                hydrogen: None,
            },
            end,
        ));
    };
    let mass = totals.mass_rate;
    // Q = −ṁ·Δh_rxn [W]: positive Δh_rxn absorbs (§5's sign convention).
    // Written as a subtraction from zero so a reaction that does nothing
    // reports +0.0, not the −0.0 a negation of a zero product gives.
    let released = 0.0 - mass * reaction.dh_rxn.value();
    // The products' enthalpy rate and capacity rate at the bed's start-of-tick
    // temperature: the outflow `H_out(T_b)` and its slope `W`.
    let outflow_at_bed = enthalpy_model
        .enthalpy_flux(slate, &reaction.products, KgPerSec(mass), bed.temperature)?
        .value();
    let capacity_rate = mass
        * enthalpy_model
            .spot_cp(slate, &reaction.products, bed.temperature)?
            .value();

    // The exact step, in `furnace_coil`'s form that never divides by `W`.
    let x = capacity_rate * dt.value() / capacity;
    let rate = released + fire + totals.enthalpy_rate - outflow_at_bed;
    let phi = if x > 0.0 { -(-x).exp_m1() / x } else { 1.0 };
    let end = start + rate * dt.value() / capacity * phi;
    let end_temperature = checked_bed_temperature(name, end, start, released + fire)?;

    // What leaves with the fluid, as an average power [W]: what came in and
    // what was released, less what the bed now stores. From the STORED change,
    // for `furnace_coil`'s reason: the bits rounding drops from `end` must not
    // reach the fluid without leaving the bed.
    let outflow = totals.enthalpy_rate + released + fire - capacity * (end - start) / dt.value();
    let outlet = enthalpy_model
        .temperature_from_enthalpy(slate, &reaction.products, outflow, mass)?
        .value();
    let fluid = checked_temperature(outlet, || {
        format!(
            "catalyst bed '{name}' outlet {outlet:.2} K is below absolute zero: {outflow:.4e} W \
             leaves with {mass:.4e} kg/s"
        )
    })?;
    Ok((
        reaction.products,
        BedPass {
            temperature: end_temperature,
            reaction_heat: Watt(released),
            residence_time,
            wall_fire,
            hydrogen: None,
        },
        fluid,
    ))
}

/// The step of a bed with a hydrogen pass (M59.1, docs/DESIGN.md §64.2): the
/// oil and the hydrogen both through the bed, both leaving at ONE temperature.
///
/// `bed_reactor`'s exact step with the two streams' capacity rates summed,
/// `C·dT_b/dt = Q + F + H_in,oil + H_in,H2 − H_out,oil(T_b) − H_out,H2(T_b)`,
/// and the outflow from the STORED change, as there. The two streams' common
/// outlet temperature is the one at which their summed sensible enthalpy is
/// that outflow — the enthalpy models are additive by mass, so it is
/// `temperature_from_enthalpy` on the two blended. What the hydrogen carried
/// off the bed, `H_in,H2 − H_out,H2(T_out)`, is reported.
///
/// **Consumption** (M59.2, docs/DESIGN.md §64.11) leaves this arithmetic as it
/// is: the whole hydrogen inflow is heated through the bed, and the share the
/// solve drew at the pass, `consumed`, is taken off the books at the outlet
/// temperature, `consumed · h(T_out)` — so the pass's outlet pipe carries the
/// rest at that temperature and both books close on the solve's own flows.
#[allow(clippy::too_many_arguments)]
fn bed_with_hydrogen(
    name: &str,
    (slate, enthalpy_model): (&Slate, &dyn EnthalpyModel),
    bed: &CatalystBed,
    (totals, feed): (InflowTotals, &Composition),
    reaction: Reaction,
    residence_time: Option<Seconds>,
    (input, factor, inflow, consumed): (HydrogenInput<'_>, f64, InflowTotals, Option<f64>),
    (fire, wall_fire): (f64, Watt),
    dt: Seconds,
) -> Result<(Composition, BedPass, Kelvin), SimError> {
    let capacity = bed.heat_capacity.value();
    let start = bed.temperature.value();
    // Each stream's inflow, its outflow at the bed's start-of-tick temperature,
    // and its capacity rate there; zero for a stream that does not flow.
    let stream =
        |totals: InflowTotals, leaving: &Composition| -> Result<(f64, f64, f64, f64), SimError> {
            match totals {
                Some(t) => Ok((
                    t.mass_rate,
                    t.enthalpy_rate,
                    enthalpy_model
                        .enthalpy_flux(slate, leaving, KgPerSec(t.mass_rate), bed.temperature)?
                        .value(),
                    t.mass_rate
                        * enthalpy_model
                            .spot_cp(slate, leaving, bed.temperature)?
                            .value(),
                )),
                None => Ok((0.0, 0.0, 0.0, 0.0)),
            }
        };
    let (oil_mass, oil_in, oil_out, oil_w) = stream(totals, &reaction.products)?;
    let (gas_mass, gas_in, gas_out, gas_w) = stream(inflow, input.composition)?;
    let released = 0.0 - oil_mass * reaction.dh_rxn.value();

    let x = (oil_w + gas_w) * dt.value() / capacity;
    let rate = released + fire + oil_in + gas_in - oil_out - gas_out;
    let phi = if x > 0.0 { -(-x).exp_m1() / x } else { 1.0 };
    let end = start + rate * dt.value() / capacity * phi;
    let end_temperature = checked_bed_temperature(name, end, start, released + fire)?;

    // Both streams' outflow from the stored change, at one temperature.
    let mass = oil_mass + gas_mass;
    let (fluid, hydrogen_heat) = if mass > 0.0 {
        let outflow = oil_in + gas_in + released + fire - capacity * (end - start) / dt.value();
        let leaving =
            Composition::blend(&reaction.products, oil_mass, input.composition, gas_mass)?;
        let outlet = enthalpy_model
            .temperature_from_enthalpy(slate, &leaving, outflow, mass)?
            .value();
        let fluid = checked_temperature(outlet, || {
            format!(
                "catalyst bed '{name}' outlet {outlet:.2} K is below absolute zero: \
                 {outflow:.4e} W leaves with {mass:.4e} kg/s of oil and hydrogen"
            )
        })?;
        let gas_leaving = enthalpy_model
            .enthalpy_flux(slate, input.composition, KgPerSec(gas_mass), fluid)?
            .value();
        (fluid, gas_in - gas_leaving)
    } else {
        (end_temperature, 0.0)
    };
    let consumption = match (input.consumption, consumed) {
        (Some(declared), Some(consumed)) => Some(pass_consumption(
            name,
            (slate, enthalpy_model),
            declared,
            (feed, &reaction.products, oil_mass),
            (input.composition, input.pressure),
            consumed,
            fluid,
        )?),
        _ => None,
    };
    Ok((
        reaction.products,
        BedPass {
            temperature: end_temperature,
            reaction_heat: Watt(released),
            residence_time,
            wall_fire,
            hydrogen: Some(HydrogenPass {
                pass: input.pass,
                factor,
                heat: Watt(hydrogen_heat),
                consumption,
            }),
        },
        fluid,
    ))
}

/// What a bed's reaction consumed of its hydrogen this tick, and what it draws
/// next tick (M59.2, docs/DESIGN.md §64.11).
///
/// - **Booked**: `consumed`, the pass's net inflow in this solve, and the
///   enthalpy it takes off the books at the outlet temperature `fluid`.
/// - **Demand**: `D = ṁ_oil · Σᵢ (y_out,i − y_in,i) · uptakeᵢ` [kg/s]. The
///   loader orders `uptake` so a reaction that only cracks makes `D ≥ 0`; a
///   negative one would be a reaction RELEASING hydrogen, which this model has
///   no account of, and is refused rather than booked.
/// - **Next rate**: `D / P` at the pressure the reaction ran at, zero at zero
///   pressure — where the factor is zero, nothing reacts and `D` is zero too.
fn pass_consumption(
    name: &str,
    (slate, enthalpy_model): (&Slate, &dyn EnthalpyModel),
    declared: &HydrogenConsumption,
    (feed, products, oil_mass): (&Composition, &Composition, f64),
    (gas, pressure): (&Composition, Pascal),
    consumed: f64,
    fluid: Kelvin,
) -> Result<PassConsumption, SimError> {
    if declared.uptake.len() != slate.len() {
        return Err(SimError::Scenario(format!(
            "catalyst bed '{name}': its hydrogen pass declares {} uptake figures for a slate of \
             {} components",
            declared.uptake.len(),
            slate.len()
        )));
    }
    // Per kilogram of each lump formed, `uptake` kilograms of hydrogen (§64.6).
    let per_kg: f64 = products
        .fractions()
        .iter()
        .zip(feed.fractions())
        .zip(&declared.uptake)
        .map(|((after, before), uptake)| (after - before) * uptake)
        .sum();
    // A ROUNDING-sized negative is zero (M59.3, docs/DESIGN.md §64.12). The
    // loader orders the figures along every path, so the exact sum is never
    // negative; computed, a bed fed products that almost nothing reacts in can
    // land a few ulps under it. The band is the terms' own scale: each of
    // `y_out` and `y_in` carries a few ulps of itself.
    let scale: f64 = products
        .fractions()
        .iter()
        .zip(feed.fractions())
        .zip(&declared.uptake)
        .map(|((after, before), uptake)| (after + before) * uptake)
        .sum();
    let band = 16.0 * f64::EPSILON * oil_mass * scale;
    let demand = match oil_mass * per_kg {
        demand if demand < 0.0 && demand >= -band => 0.0,
        demand => demand,
    };
    if !demand.is_finite() || demand < 0.0 {
        return Err(SimError::Numerical(format!(
            "catalyst bed '{name}': its reaction would consume {demand:.4e} kg/s of hydrogen. A \
             negative figure is a reaction releasing hydrogen, which the uptake table's order \
             rules out for one that only cracks (docs/DESIGN.md §64.11)"
        )));
    }
    let rate = if pressure.value() > 0.0 {
        demand / pressure.value()
    } else {
        0.0
    };
    if !rate.is_finite() {
        return Err(SimError::Numerical(format!(
            "catalyst bed '{name}': its hydrogen sink's rate {rate} is not finite at {:.4e} Pa",
            pressure.value()
        )));
    }
    let consumed_heat = enthalpy_model
        .enthalpy_flux(slate, gas, KgPerSec(consumed), fluid)?
        .value();
    Ok(PassConsumption {
        consumed: KgPerSec(consumed),
        consumed_heat: Watt(consumed_heat),
        demand: KgPerSec(demand),
        rate,
    })
}

/// Net mass flow INTO `node` [kg/s] over its incident edges, in the solve's own
/// flows. Self-loops carry nothing anywhere and are skipped, as `inflow_edges`
/// skips them.
fn net_inflow(graph: &PlantGraph, edge_mass_flow: &BTreeMap<EdgeId, f64>, node: NodeId) -> f64 {
    let mut into = 0.0;
    for (edge, other, incoming) in graph.incident(node) {
        if other == node {
            continue;
        }
        let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
        into += if incoming { flow } else { -flow };
    }
    into
}

/// A bed's end-of-tick temperature, refused if it is not finite or below 0 K.
fn checked_bed_temperature(
    name: &str,
    end: f64,
    start: f64,
    heat: f64,
) -> Result<Kelvin, SimError> {
    if !end.is_finite() {
        return Err(SimError::Numerical(format!(
            "catalyst bed '{name}' temperature is not finite ({end}) after a tick from \
             {start:.2} K with {heat:.4e} W released and fired into it"
        )));
    }
    checked_temperature(end, || {
        format!(
            "catalyst bed '{name}' cools to {end:.2} K, below absolute zero, from {start:.2} K \
             with {heat:.4e} W released and fired into it"
        )
    })
}

/// Both outlet temperatures of a coupled `HeatExchanger` pair [K].
///
/// ΔT-effectiveness fidelity (docs/DESIGN.md §4a) — no NTU, no LMTD, no
/// counter- versus co-current distinction at this level:
///
/// ```text
/// C_a = ṁ_a·cp_a,  C_b = ṁ_b·cp_b,  C_min = min(C_a, C_b)
/// Q   = ε·C_min·(T_a_in − T_b_in)
/// T_a_out = T_a_in − Q/C_a        T_b_out = T_b_in + Q/C_b
/// ```
///
/// Three properties this arrangement buys, each deliberate:
///
/// - **One signed `Q`, subtracted from A and added to B.** Energy conserves by
///   construction, for any ε and any pair of capacity rates — there is no
///   balance left to get wrong. Computing each side's outlet from its own
///   independent effectiveness term is the natural-looking alternative and
///   quietly creates or destroys heat.
/// - **Neither side is the hot one.** The sign of `T_a_in − T_b_in` decides the
///   direction, so a service that reverses needs no reconfiguration and no
///   second code path.
/// - **`C_min`, not `C_max`.** With ε ≤ 1 this bounds `Q` by the heat the
///   smaller stream can actually carry, so the outlets cannot cross and the
///   second law holds without a check. The two agree exactly when the capacity
///   rates are equal, which is why the reference case makes them unequal.
///
/// A side with no throughput exchanges nothing: `Q` is zero and each side falls
/// back to the ordinary zero-volume rules. That is physics, not a guard — an
/// exchanger with one stream stopped is a pipe.
#[allow(clippy::too_many_arguments)]
fn exchange_pair(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy_model: &dyn EnthalpyModel,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    edge_dissipation: &BTreeMap<EdgeId, Watt>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    separations: &BTreeMap<NodeId, Separation>,
    previous: &BTreeMap<NodeId, Kelvin>,
    (side_a, side_b): (NodeId, NodeId),
) -> Result<(Kelvin, Kelvin), SimError> {
    let (_, effectiveness) = graph.exchanger_partner(side_a).ok_or_else(|| {
        SimError::Numerical(format!(
            "internal: '{}' was swept as an exchanger side but has no coupling",
            graph.node(side_a).name
        ))
    })?;

    let totals_a = inflow_totals(
        graph,
        slate,
        enthalpy_model,
        edge_mass_flow,
        edge_dissipation,
        temperature,
        composition,
        separations,
        side_a,
    )?;
    let totals_b = inflow_totals(
        graph,
        slate,
        enthalpy_model,
        edge_mass_flow,
        edge_dissipation,
        temperature,
        composition,
        separations,
        side_b,
    )?;

    // Positive duty = heat flowing A → B.
    //
    // `C_min` is a capacity RATE and stays one under a shape — the second reader
    // of `inflow_totals`' capacity sum, and the one §20's consumer enumeration
    // does not name. It is each side's SPOT capacity at its own inlet, for the
    // reason `stream_cp_at` gives: the mean this would rather have is over an
    // outlet that does not exist until the duty this is sizing is known.
    let duty = match (totals_a, totals_b) {
        (Some(a), Some(b)) => {
            let inlet_a = enthalpy_model
                .mix_temperature(slate, mixed_composition_at(composition, side_a)?, a)?
                .value();
            let inlet_b = enthalpy_model
                .mix_temperature(slate, mixed_composition_at(composition, side_b)?, b)?
                .value();
            effectiveness * a.capacity_rate.min(b.capacity_rate) * (inlet_a - inlet_b)
        }
        _ => 0.0,
    };

    Ok((
        exchanger_side_outlet(
            graph,
            slate,
            enthalpy_model,
            composition,
            previous,
            side_a,
            totals_a,
            -duty,
        )?,
        exchanger_side_outlet(
            graph,
            slate,
            enthalpy_model,
            composition,
            previous,
            side_b,
            totals_b,
            duty,
        )?,
    ))
}

/// One exchanger side's outlet [K]: its own inflow mix, plus whatever heat it
/// receives — `transferred` from the partner stream, and `heat_load` from a
/// fire, which stacks here exactly as it does on a furnace.
#[allow(clippy::too_many_arguments)]
fn exchanger_side_outlet(
    graph: &PlantGraph,
    slate: &Slate,
    enthalpy_model: &dyn EnthalpyModel,
    composition: &BTreeMap<NodeId, Composition>,
    previous: &BTreeMap<NodeId, Kelvin>,
    node: NodeId,
    totals: InflowTotals,
    transferred: f64,
) -> Result<Kelvin, SimError> {
    let Some(totals) = totals else {
        // Same indeterminate-but-inert case `mix_inflows` documents.
        return Ok(previous.get(&node).copied().unwrap_or(T_AMBIENT));
    };
    let (enthalpy, capacity) = (totals.enthalpy_rate, totals.capacity_rate);
    let heat = heat_load(graph.node(node)).value() + transferred;
    let outlet = enthalpy_model
        .mix_temperature(
            slate,
            mixed_composition_at(composition, node)?,
            InflowEnthalpy {
                enthalpy_rate: enthalpy + heat,
                ..totals
            },
        )?
        .value();
    checked_temperature(outlet, || {
        format!(
            "exchanger side '{}' cools to {outlet:.2} K, below absolute zero: it \
             gives up {:.4e} W to its partner stream (plus {:.4e} W of external \
             heat), more than the {capacity:.4e} W/K · {:.2} K of sensible heat \
             its inflow carries above 0 K.",
            graph.node(node).name,
            -transferred,
            heat_load(graph.node(node)).value(),
            enthalpy / capacity + T_REF.value(),
        )
    })
}

#[cfg(test)]
mod tests {
    //! The sweep is tested with HAND-BUILT flow maps, never through a solver.
    //! Its contract is "given these flows, produce these temperatures", so
    //! feeding it flows directly tests exactly that — and lets the recycle case
    //! be built on demand instead of hoping a network happens to circulate.

    use super::*;
    use crate::components::{Composition, Slate};
    use crate::graph::{HeatExchangerCoupling, LeakRole, Node, Pipe, TankState};
    use crate::stream::Stream;
    use crate::traits::Reaction;
    use crate::units::*;

    /// **A condensing curve pinches inside a water cooler, and the zone duty
    /// stops there** (M62.4b, docs/DESIGN.md §67.13). A hand-built hot side:
    /// 1 MW of superheated vapour from 150 °C to its dew point at 100 °C
    /// (0.02 MW/K), 8 MW condensing at 100 °C, its liquid then cooling at
    /// 0.1 MW/K; against 0.2 MW/K of water from 30 °C, counter-current. The
    /// ceiling — the hot side cooled to 30 °C — is 16 MW, but the water reaches
    /// 100 °C at the dew point once the duty passes 15 MW: the pinch, inside the
    /// exchanger. With unlimited area the duty stops at the pinch the zone
    /// boundaries see (between 15 MW and 15.56 MW, where the first boundary past
    /// the dew point crosses), never at the ceiling; a modest area takes
    /// less, and every boundary's approach stays positive.
    #[test]
    fn a_zone_duty_stops_at_an_internal_pinch() {
        let arriving = 16.0e6;
        let hot_at = |energy: f64| -> Result<f64, SimError> {
            let x = arriving - energy;
            Ok(if x <= 1.0e6 {
                423.15 - x / 0.02e6
            } else if x <= 9.0e6 {
                373.15
            } else {
                373.15 - (x - 9.0e6) / 0.1e6
            })
        };
        let water = 0.2e6;
        let ceiling = 16.0e6;
        let unlimited = zone_duty(
            "pinch",
            arriving,
            423.15,
            ceiling,
            (1e15, water, 303.15),
            hot_at,
        )
        .unwrap();
        assert!(
            unlimited > 15.0e6 && unlimited < 15.6e6,
            "unlimited area: {unlimited} W, not at the pinch"
        );
        for ua in [1e15, 2.0e5] {
            let duty = zone_duty(
                "pinch",
                arriving,
                423.15,
                ceiling,
                (ua, water, 303.15),
                hot_at,
            )
            .unwrap();
            let zones = WATER_COOLER_ZONES;
            for j in 0..=zones {
                let hot = hot_at(arriving - duty * j as f64 / zones as f64).unwrap();
                let cold = 303.15 + duty * (zones - j) as f64 / (zones as f64 * water);
                assert!(
                    hot > cold,
                    "UA {ua}: boundary {j} crosses ({hot} K against {cold} K)"
                );
            }
            assert!(
                duty <= unlimited,
                "UA {ua}: {duty} W past the unlimited {unlimited} W"
            );
        }
    }

    /// Identity reaction for the sweep tests that carry no reactor — mirrors the
    /// `solvers::NoReactions` the engine uses, kept here so `core`'s own tests
    /// depend on nothing downstream. Reactor-specific tests define their own
    /// converting model inline.
    struct NoRxn;
    impl ReactionModel for NoRxn {
        fn name(&self) -> &'static str {
            "test-none"
        }
        fn react(
            &self,
            feed: &Composition,
            _t: Kelvin,
            _tau: Seconds,
            _slate: &Slate,
        ) -> Result<Reaction, SimError> {
            Ok(Reaction {
                products: feed.clone(),
                dh_rxn: JPerKg::ZERO,
            })
        }
    }

    /// The separation model for sweep tests, and it REFUSES to separate.
    ///
    /// `core`'s own tests build no column — the splitter's math and its hand
    /// calculations moved to `solvers::separation` with the code in M7.1 — so a
    /// stub that returns the right shape would be a second copy of a rule that
    /// must agree with the real one, with nothing forcing it to. This one cannot
    /// drift because it computes nothing; if a test ever does sweep a column, it
    /// fails loudly here instead of silently grading itself against a duplicate.
    /// The constant-capacity enthalpy model, mirroring
    /// `solvers::ConstantEnthalpy`, and here for the reason `TestThermo` and
    /// `NoRxn` are: `core`'s own tests depend on nothing downstream.
    ///
    /// **It is a second copy of an expression fork 1 exists to keep single, and
    /// that is stated rather than hidden.** What bounds the drift is that this
    /// copy has no readers outside this module and grades no shipped plant: every
    /// wired scenario runs `solvers::ConstantEnthalpy`, and the regression corpus
    /// is what says that model reproduces the pre-M16 arithmetic. A drift here
    /// would fail these tests loudly, because the temperatures they assert are
    /// hand calculations rather than the model's own output.
    /// The line flash every pre-M53 sweep test means: no vapour, the mix handed
    /// back untouched (M53, docs/DESIGN.md §58).
    struct NoLineFlashStub;

    impl crate::traits::LineFlashModel for NoLineFlashStub {
        fn name(&self) -> &'static str {
            "none-stub"
        }
        fn carries_vapour(&self) -> bool {
            false
        }
        fn supply(
            &self,
            _: &Slate,
            _: &Composition,
            _: Kelvin,
            _: crate::units::Pascal,
            _: &dyn ThermoModel,
            _: &dyn EnthalpyModel,
        ) -> Result<Option<crate::traits::VapourShare>, SimError> {
            Ok(None)
        }
        fn settle(
            &self,
            _: &Slate,
            _: &Composition,
            liquid_equivalent: Kelvin,
            _: crate::units::Pascal,
            _: &dyn ThermoModel,
            _: &dyn EnthalpyModel,
        ) -> Result<crate::traits::Settled, SimError> {
            Ok(crate::traits::Settled {
                temperature: liquid_equivalent,
                vapour: None,
            })
        }
        fn density(
            &self,
            _: &Slate,
            _: &Composition,
            _: Kelvin,
            _: crate::units::Pascal,
            _: &dyn ThermoModel,
            _: &dyn EnthalpyModel,
        ) -> Result<Option<crate::traits::TwoPhaseDensity>, SimError> {
            Ok(None)
        }
    }

    /// Every node at atmospheric pressure: what a sweep test with no flow solve
    /// hands the line flash, which `NoLineFlashStub` never reads.
    fn everywhere_atmospheric(graph: &PlantGraph) -> BTreeMap<NodeId, crate::units::Pascal> {
        graph
            .node_ids()
            .map(|id| (id, crate::units::P_ATM))
            .collect()
    }

    struct ConstantEnthalpyStub;
    impl EnthalpyModel for ConstantEnthalpyStub {
        fn name(&self) -> &'static str {
            "test-constant-cp"
        }
        fn specific_enthalpy(
            &self,
            slate: &Slate,
            composition: &Composition,
            temperature: Kelvin,
        ) -> Result<JPerKg, SimError> {
            let cp = composition.mixture_cp(slate).value();
            Ok(JPerKg(cp * (temperature.value() - T_REF.value())))
        }
        fn enthalpy_flux(
            &self,
            slate: &Slate,
            composition: &Composition,
            mass_flow: KgPerSec,
            temperature: Kelvin,
        ) -> Result<Watt, SimError> {
            let cp = composition.mixture_cp(slate).value();
            Ok(Watt(
                mass_flow.value() * cp * (temperature.value() - T_REF.value()),
            ))
        }
        fn enthalpy_stock(
            &self,
            slate: &Slate,
            composition: &Composition,
            mass: crate::units::Kg,
            temperature: Kelvin,
        ) -> Result<f64, SimError> {
            let cp = composition.mixture_cp(slate).value();
            Ok(mass.value() * cp * (temperature.value() - T_REF.value()))
        }
        fn mean_cp(
            &self,
            slate: &Slate,
            composition: &Composition,
            _t1: Kelvin,
            _t2: Kelvin,
        ) -> Result<JPerKgK, SimError> {
            Ok(composition.mixture_cp(slate))
        }
        fn spot_cp(
            &self,
            slate: &Slate,
            composition: &Composition,
            _temperature: Kelvin,
        ) -> Result<JPerKgK, SimError> {
            Ok(composition.mixture_cp(slate))
        }
        fn specific_internal_energy(
            &self,
            slate: &Slate,
            composition: &Composition,
            temperature: Kelvin,
        ) -> Result<JPerKg, SimError> {
            let cv = composition.mixture_cv(slate).value();
            let cp = composition.mixture_cp(slate).value();
            Ok(JPerKg(cv * temperature.value() - cp * T_REF.value()))
        }
        fn temperature_from_enthalpy(
            &self,
            slate: &Slate,
            composition: &Composition,
            energy: f64,
            mass: f64,
        ) -> Result<Kelvin, SimError> {
            let cp = composition.mixture_cp(slate).value();
            Ok(Kelvin(T_REF.value() + energy / (mass * cp)))
        }
        fn temperature_from_internal_energy(
            &self,
            slate: &Slate,
            composition: &Composition,
            energy: f64,
            mass: f64,
        ) -> Result<Kelvin, SimError> {
            let cv = composition.mixture_cv(slate).value();
            let cp = composition.mixture_cp(slate).value();
            Ok(Kelvin((energy / mass + cp * T_REF.value()) / cv))
        }
        fn mix_temperature(
            &self,
            _slate: &Slate,
            _composition: &Composition,
            totals: InflowEnthalpy,
        ) -> Result<Kelvin, SimError> {
            Ok(Kelvin(
                T_REF.value() + totals.enthalpy_rate / totals.capacity_rate,
            ))
        }
    }

    struct NoSeparation;
    impl SeparationModel for NoSeparation {
        fn name(&self) -> &'static str {
            "test-refuses"
        }
        fn separate(
            &self,
            _pass: &ColumnPass<'_>,
            _thermo: &dyn ThermoModel,
        ) -> Result<Separation, SimError> {
            Err(SimError::Numerical(
                "this sweep test builds no column; separation belongs to `solvers`".into(),
            ))
        }
    }

    /// Property stub, mirroring `solvers::ConstantThermo`. It is here because
    /// `resolve_node_states` hands it to the separation model (DESIGN §5,
    /// correction 2); no sweep test builds a column, so its `k_value` is
    /// unreachable and refuses rather than inventing a number, exactly as the
    /// real constant fidelity does.
    struct TestThermo;
    impl ThermoModel for TestThermo {
        fn name(&self) -> &'static str {
            "test-constant"
        }
        fn k_value(
            &self,
            _slate: &Slate,
            _component: usize,
            _temperature: Kelvin,
            _pressure: Pascal,
        ) -> Result<f64, SimError> {
            Err(SimError::Numerical(
                "this sweep test has no phase equilibrium; K-values belong to `solvers`".into(),
            ))
        }
        fn dh_vap(
            &self,
            _slate: &Slate,
            _component: usize,
            _temperature: Kelvin,
        ) -> Result<JPerMol, SimError> {
            Err(SimError::Numerical(
                "this sweep test has no vapour phase; latent heats belong to `solvers`".into(),
            ))
        }
        /// `Scenario`, matching `solvers::ConstantThermo` rather than the two
        /// arms above: this is the variant the engine's cavitation pass reads as
        /// "no criterion here", and a stub that refused with `Numerical` would
        /// make a sweep test fail a tick instead of reporting nothing
        /// (docs/DESIGN.md §13).
        fn bubble_pressure(
            &self,
            _slate: &Slate,
            _composition: &Composition,
            _temperature: Kelvin,
        ) -> Result<Pascal, SimError> {
            Err(SimError::Scenario(
                "this sweep test has no phase equilibrium; bubble pressures belong to \
                 `solvers`"
                    .into(),
            ))
        }
    }

    fn node(name: &str, kind: NodeKind) -> Node {
        Node {
            name: name.into(),
            kind,
            heat_input: Watt::ZERO,
        }
    }

    fn source(name: &str, temperature: Kelvin) -> Node {
        node(
            name,
            NodeKind::Source {
                pressure: Pascal(2.0e5),
                temperature,
                composition: Composition::pure(1, 0),
            },
        )
    }

    fn tank(name: &str, temperature: Kelvin) -> Node {
        node(
            name,
            NodeKind::Tank(TankState {
                area: SquareMeter(10.0),
                height: Meter(10.0),
                mass: Kg(50_000.0),
                temperature,
                composition: Composition::pure(1, 0),
                ambient_ua: WattPerKelvin::ZERO,
            }),
        )
    }

    fn pipe(name: &str) -> Pipe {
        Pipe {
            name: name.into(),
            length: Meter(10.0),
            diameter: Meter(0.1),
            friction_factor: 0.02,
            elevation_change: Meter(0.0),
            leak: LeakRole::None,
            ambient_ua: WattPerKelvin::ZERO,
            stream: Stream::stagnant(1, T_AMBIENT, P_ATM),
        }
    }

    /// No frictional dissipation on any edge.
    ///
    /// The right input for every test in this module: they hand-build a flow map
    /// to exercise the sweep's TOPOLOGY — upwind direction, Kahn ordering, the
    /// exchanger merge, the recycle rejection — none of which friction touches,
    /// and none of which has a `QuadraticBranch` behind it to compute an honest
    /// `Φ` from anyway. Dissipation is pinned where it is produced, on a real
    /// solve, by `scenarios/tests/dissipation_reference.rs`; the transform itself
    /// is pinned by the `pipe_outlet_temperature` cases below, which pass `Φ`
    /// explicitly.
    ///
    /// Deliberately a test fixture and NOT a constructor on the production API.
    /// DESIGN §3a's whole argument for this term is that it has no free parameter
    /// and therefore no honest "off" switch; an `EdgeSolution::frictionless()` in
    /// `core` would be exactly that switch wearing a helper's clothes.
    fn no_friction() -> BTreeMap<EdgeId, Watt> {
        BTreeMap::new()
    }

    fn resolve(
        graph: &PlantGraph,
        flows: &BTreeMap<EdgeId, f64>,
    ) -> Result<BTreeMap<NodeId, Kelvin>, SimError> {
        resolve_node_states(
            graph,
            &Slate::water_only(),
            flows,
            &no_friction(),
            &NoRxn,
            &NoSeparation,
            &TestThermo,
            &ConstantEnthalpyStub,
            &NoLineFlashStub,
            &everywhere_atmospheric(graph),
            &NodeStates::default(),
            &BTreeMap::new(),
            Seconds(1.0),
        )
        .map(|states| states.temperature)
    }

    /// The composition half of the same sweep, for the tests that read it.
    fn resolve_compositions(
        graph: &PlantGraph,
        slate: &Slate,
        flows: &BTreeMap<EdgeId, f64>,
    ) -> Result<BTreeMap<NodeId, Composition>, SimError> {
        resolve_node_states(
            graph,
            slate,
            flows,
            &no_friction(),
            &NoRxn,
            &NoSeparation,
            &TestThermo,
            &ConstantEnthalpyStub,
            &NoLineFlashStub,
            &everywhere_atmospheric(graph),
            &NodeStates::default(),
            &BTreeMap::new(),
            Seconds(1.0),
        )
        .map(|states| states.composition)
    }

    /// The composition sweep, on a slate whose two cuts have DELIBERATELY
    /// mismatched `cp` — the property that separates a mass-weighted mix from a
    /// capacity-rate-weighted one.
    mod composition_mixing {
        use super::*;
        use crate::components::{Phase, PseudoComponent};

        fn cut(name: &str, cp: f64) -> PseudoComponent {
            PseudoComponent {
                name: name.into(),
                tb: Kelvin(400.0),
                molar_mass: KgPerMol(0.1),
                density: Some(KgPerM3(800.0)),
                cp: JPerKgK(cp),
                cp_shape: None,
                phase: Phase::Liquid,
                dh_vap: None,
                cp_vapour: None,
                free_water: false,
            }
        }

        /// cp ratio 4:1. Every other property is equal, so nothing but the
        /// weighting rule can move the answer.
        fn two_cuts() -> Slate {
            Slate::new(vec![cut("light", 1000.0), cut("heavy", 4000.0)]).unwrap()
        }

        fn feed(name: &str, fractions: &[f64]) -> Node {
            node(
                name,
                NodeKind::Source {
                    pressure: Pascal(2.0e5),
                    temperature: Kelvin(300.0),
                    composition: Composition::from_weights(fractions).unwrap(),
                },
            )
        }

        /// REFERENCE — the blend, predicted from the MASS RATIO by hand.
        ///
        /// A junction is fed 1 kg/s of pure `light` and 3 kg/s of pure `heavy`.
        /// Per-component mass balance on a vessel with no accumulation:
        ///
        ///   light: 1 kg/s in, of 4 kg/s total  ⇒  f_light = 0.25
        ///   heavy: 3 kg/s in, of 4 kg/s total  ⇒  f_heavy = 0.75
        ///
        /// stated from the flows alone, with no reference to `mix_compositions`
        /// or to `Composition::blend` — the expected numbers cannot be produced
        /// by the same rule they are checking.
        ///
        /// THE MUTATION THIS EXISTS FOR: weight the mix by `ṁ·cp` instead of
        /// `ṁ`, i.e. reuse the enthalpy weights sitting in the same sweep. The
        /// capacity rates here are 1·1000 and 3·4000, so that mutation returns
        /// f_light = 1000/13000 ≈ 0.0769 rather than 0.25. It is a mutation
        /// worth guarding at this size because it is nearly invisible
        /// elsewhere: total mass is still 4 kg/s, every energy balance still
        /// closes, and only the fractions are wrong.
        ///
        /// The 4:1 cp ratio is what gives the two answers room to differ. With
        /// equal cp they coincide exactly and this test would pass under the
        /// mutation — which is the whole reason the slate above is built by
        /// hand instead of reusing water.
        #[test]
        fn a_junction_blends_its_inflows_by_mass_not_by_capacity_rate() {
            let slate = two_cuts();
            let mut g = PlantGraph::new();
            let light_src = g.add_node(feed("light_src", &[1.0, 0.0]));
            let heavy_src = g.add_node(feed("heavy_src", &[0.0, 1.0]));
            let mix = g.add_node(node("mix", NodeKind::Junction));
            let e_light = g.add_pipe(light_src, mix, pipe("light_line"));
            let e_heavy = g.add_pipe(heavy_src, mix, pipe("heavy_line"));

            let flows = BTreeMap::from([(e_light, 1.0), (e_heavy, 3.0)]);
            let composition = resolve_compositions(&g, &slate, &flows).unwrap();

            let mixed = composition[&mix].fractions();
            assert!(
                (mixed[0] - 0.25).abs() < 1e-12 && (mixed[1] - 0.75).abs() < 1e-12,
                "1 kg/s light + 3 kg/s heavy is 25/75 by MASS; got {mixed:?}. The \
                 capacity-rate weighting this guards against gives ~[0.077, 0.923]."
            );
        }

        /// The consistency that lets composition ride the energy sweep without
        /// perturbing it: cp is linear in composition, so the mixed stream's
        /// capacity rate equals the sum of its inflows'.
        ///
        ///   Σ_c (Σ_s ṁ_s·f_sc)·cp_c  =  Σ_s ṁ_s·cp_s
        ///   here: 4 kg/s · (0.25·1000 + 0.75·4000) = 13 000 W/K = 1·1000 + 3·4000
        ///
        /// Stated as a test because it is the reason the two mixes can disagree
        /// about their weights and still describe the same stream. If it failed,
        /// M2's energy balances would drift the moment a slate had two cuts.
        #[test]
        fn the_mixed_capacity_rate_is_the_sum_of_the_inflows() {
            let slate = two_cuts();
            let mut g = PlantGraph::new();
            let light_src = g.add_node(feed("light_src", &[1.0, 0.0]));
            let heavy_src = g.add_node(feed("heavy_src", &[0.0, 1.0]));
            let mix = g.add_node(node("mix", NodeKind::Junction));
            let e_light = g.add_pipe(light_src, mix, pipe("light_line"));
            let e_heavy = g.add_pipe(heavy_src, mix, pipe("heavy_line"));

            let flows = BTreeMap::from([(e_light, 1.0), (e_heavy, 3.0)]);
            let composition = resolve_compositions(&g, &slate, &flows).unwrap();

            let mixed_capacity = 4.0 * composition[&mix].mixture_cp(&slate).value();
            let inflow_capacity = 1.0 * 1000.0 + 3.0 * 4000.0;
            assert!(
                (mixed_capacity - inflow_capacity).abs() < 1e-9,
                "{mixed_capacity} W/K out vs {inflow_capacity} W/K in"
            );
        }

        /// Composition follows the FLOW's direction, not the edge's. Both pipes
        /// below are stored pointing away from the junction and carry negative
        /// flow, so the fluid arrives from the far ends — and a sweep reading
        /// `from` unconditionally would take the junction's own (unresolved)
        /// composition as its inlet.
        ///
        /// This is the composition twin of the reverse-flow temperature case,
        /// and it needs its own gate: the two fields read the upwind end through
        /// one helper now, but nothing in the type system stops that from being
        /// re-derived independently later.
        #[test]
        fn reverse_flow_blends_from_the_far_end() {
            let slate = two_cuts();
            let mut g = PlantGraph::new();
            let mix = g.add_node(node("mix", NodeKind::Junction));
            let light_src = g.add_node(feed("light_src", &[1.0, 0.0]));
            let heavy_src = g.add_node(feed("heavy_src", &[0.0, 1.0]));
            // Stored mix → source, flowing source → mix.
            let e_light = g.add_pipe(mix, light_src, pipe("light_line"));
            let e_heavy = g.add_pipe(mix, heavy_src, pipe("heavy_line"));

            let flows = BTreeMap::from([(e_light, -1.0), (e_heavy, -3.0)]);
            let composition = resolve_compositions(&g, &slate, &flows).unwrap();

            let mixed = composition[&mix].fractions();
            assert!(
                (mixed[0] - 0.25).abs() < 1e-12,
                "reverse flow must blend the same 25/75 as forward flow, got {mixed:?}"
            );
        }

        /// A junction with nothing flowing through it holds a composition rather
        /// than dividing 0/0 — the rule `mix_inflows` applies to temperature,
        /// mirrored. `Composition::from_weights` rejects an all-zero weight
        /// vector, so without the explicit fallback this path would ERROR on a
        /// closed valve, which is an ordinary plant state.
        #[test]
        fn a_stagnant_junction_holds_a_valid_composition() {
            let slate = two_cuts();
            let mut g = PlantGraph::new();
            let src = g.add_node(feed("src", &[0.5, 0.5]));
            let idle = g.add_node(node("idle", NodeKind::Junction));
            let dead = g.add_pipe(src, idle, pipe("dead_leg"));

            let flows = BTreeMap::from([(dead, 0.0)]);

            // With history, it holds what it last saw.
            let previous = NodeStates {
                temperature: BTreeMap::new(),
                composition: BTreeMap::from([(
                    idle,
                    Composition::from_weights(&[0.2, 0.8]).unwrap(),
                )]),
                ..Default::default()
            };
            let held = resolve_node_states(
                &g,
                &slate,
                &flows,
                &no_friction(),
                &NoRxn,
                &NoSeparation,
                &TestThermo,
                &ConstantEnthalpyStub,
                &NoLineFlashStub,
                &everywhere_atmospheric(&g),
                &previous,
                &BTreeMap::new(),
                Seconds(1.0),
            )
            .unwrap()
            .composition;
            assert_eq!(held[&idle].fractions(), &[0.2, 0.8]);

            // With none, a valid composition rather than an error or a NaN.
            let fresh = resolve_compositions(&g, &slate, &flows).unwrap();
            let sum: f64 = fresh[&idle].fractions().iter().sum();
            assert!((sum - 1.0).abs() < 1e-12, "must still sum to 1");
        }
    }

    /// Enthalpy weighting, hand-calculated: 1 kg/s at 280 K and 3 kg/s at 320 K
    /// mix to (1·280 + 3·320)/4 = 310 K exactly (equal cp ⇒ mass-weighted mean).
    #[test]
    fn a_junction_mixes_its_inflows_by_enthalpy() {
        let mut g = PlantGraph::new();
        let cold = g.add_node(source("cold", Kelvin(280.0)));
        let hot = g.add_node(source("hot", Kelvin(320.0)));
        let mix = g.add_node(node("mix", NodeKind::Junction));
        let out = g.add_node(node(
            "out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let e_cold = g.add_pipe(cold, mix, pipe("cold_line"));
        let e_hot = g.add_pipe(hot, mix, pipe("hot_line"));
        let e_out = g.add_pipe(mix, out, pipe("outlet"));

        let flows = BTreeMap::from([(e_cold, 1.0), (e_hot, 3.0), (e_out, 4.0)]);
        let temperature = resolve(&g, &flows).expect("an acyclic network must resolve");

        assert!(
            (temperature[&mix].value() - 310.0).abs() < 1e-12,
            "expected the 1:3 mix of 280 K and 320 K to be 310 K, got {}",
            temperature[&mix].value()
        );
    }

    /// Upwind is chosen by FLOW SIGN, not by the graph's edge direction: with
    /// the flow reversed, the junction must take the sink's temperature — the
    /// node the pipe points AT. A model that trusted edge direction would read
    /// the source and report 280 K.
    #[test]
    fn upwind_follows_the_flow_not_the_edge_direction() {
        let mut g = PlantGraph::new();
        let src = g.add_node(source("src", Kelvin(280.0)));
        let mix = g.add_node(node("mix", NodeKind::Junction));
        let back = g.add_node(node(
            "back",
            NodeKind::Sink {
                pressure: Pascal(9.0e5),
                temperature: Kelvin(350.0),
                composition: Composition::pure(1, 0),
            },
        ));
        let e_in = g.add_pipe(src, mix, pipe("inlet"));
        let e_out = g.add_pipe(mix, back, pipe("outlet"));

        // Both edges run backwards: the sink back-feeds through `mix` to `src`.
        let flows = BTreeMap::from([(e_in, -2.0), (e_out, -2.0)]);
        let temperature = resolve(&g, &flows).expect("an acyclic network must resolve");

        assert!(
            (temperature[&mix].value() - 350.0).abs() < 1e-12,
            "reverse flow must carry the sink's 350 K to the junction, got {}",
            temperature[&mix].value()
        );
    }

    /// Q into a zero-volume node with throughput: first law for a heater,
    /// ΔT = Q/(ṁ·cp). 2 kg/s of water and 41 840 W ⇒ exactly +5 K.
    #[test]
    fn heat_input_on_a_junction_raises_its_outlet_temperature() {
        let mut g = PlantGraph::new();
        let src = g.add_node(source("src", Kelvin(300.0)));
        let heater = g.add_node(Node {
            name: "heater".into(),
            kind: NodeKind::Junction,
            heat_input: Watt(2.0 * 4184.0 * 5.0),
        });
        let out = g.add_node(node(
            "out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let e_in = g.add_pipe(src, heater, pipe("inlet"));
        let e_out = g.add_pipe(heater, out, pipe("outlet"));

        let flows = BTreeMap::from([(e_in, 2.0), (e_out, 2.0)]);
        let temperature = resolve(&g, &flows).expect("an acyclic network must resolve");

        assert!(
            (temperature[&heater].value() - 305.0).abs() < 1e-12,
            "Q/(ṁ·cp) must raise 300 K by exactly 5 K, got {}",
            temperature[&heater].value()
        );
    }

    /// A ring of junctions with flow circulating: each one's temperature depends
    /// on the previous, and no inertial node breaks the chain. The sweep must
    /// say so rather than silently resolving in an arbitrary order.
    #[test]
    fn zero_volume_recycle_is_rejected() {
        let mut g = PlantGraph::new();
        let a = g.add_node(node("j_a", NodeKind::Junction));
        let b = g.add_node(node("j_b", NodeKind::Junction));
        let c = g.add_node(node("j_c", NodeKind::Junction));
        let ab = g.add_pipe(a, b, pipe("ab"));
        let bc = g.add_pipe(b, c, pipe("bc"));
        let ca = g.add_pipe(c, a, pipe("ca"));

        let flows = BTreeMap::from([(ab, 1.0), (bc, 1.0), (ca, 1.0)]);
        let err = resolve(&g, &flows).expect_err("a zero-volume-only recycle must be rejected");

        let message = err.to_string();
        assert!(
            message.contains("recycle"),
            "the error must explain what went wrong, got: {message}"
        );
        for name in ["j_a", "j_b", "j_c"] {
            assert!(
                message.contains(name),
                "the error must name the stuck node {name} for diagnosis, got: {message}"
            );
        }
    }

    /// The contrast case that proves the rejection above is about ZERO-VOLUME
    /// recycles specifically, not about loops. The same ring with a tank spliced
    /// in resolves fine: the tank's temperature is a start-of-tick constant, so
    /// the dependency chain terminates — and its 350 K propagates right round.
    #[test]
    fn a_tank_in_the_loop_breaks_the_recycle() {
        let mut g = PlantGraph::new();
        let a = g.add_node(node("j_a", NodeKind::Junction));
        let vessel = g.add_node(tank("vessel", Kelvin(350.0)));
        let c = g.add_node(node("j_c", NodeKind::Junction));
        let ab = g.add_pipe(a, vessel, pipe("ab"));
        let bc = g.add_pipe(vessel, c, pipe("bc"));
        let ca = g.add_pipe(c, a, pipe("ca"));

        let flows = BTreeMap::from([(ab, 1.0), (bc, 1.0), (ca, 1.0)]);
        let temperature = resolve(&g, &flows).expect("a tank in the loop must break the cycle");

        for (id, name) in [(a, "j_a"), (c, "j_c")] {
            assert!(
                (temperature[&id].value() - 350.0).abs() < 1e-12,
                "{name} must inherit the tank's 350 K, got {}",
                temperature[&id].value()
            );
        }
    }

    /// The shared guard, tested directly rather than only through the callers
    /// that reach it. Every path that computes a temperature routes through
    /// this one function, so its contract is worth pinning independently of
    /// which levers happen to reach it. That set moves: a negative
    /// `Command::SetHeatInput` was one until the command started refusing it,
    /// a cooler duty is one now, ambient exchange will be one later. This test
    /// holds whatever the plant can currently do to a node.
    ///
    /// 0 K itself is legal: absolute zero is unreachable, not forbidden, and
    /// erroring on it would reject an exactly-drained stream at the boundary.
    #[test]
    fn checked_temperature_rejects_below_absolute_zero_only() {
        assert_eq!(
            checked_temperature(0.0, || "unused".into())
                .expect("0 K is a legal boundary, not an error")
                .value(),
            0.0
        );
        assert_eq!(
            checked_temperature(300.0, || "unused".into())
                .expect("an ordinary temperature must pass")
                .value(),
            300.0
        );

        let err = checked_temperature(-1e-9, || "the context explains it".into())
            .expect_err("any negative absolute temperature must be rejected, however small");
        assert!(
            err.to_string().contains("the context explains it"),
            "the caller's diagnostic must reach the error, got: {err}"
        );
    }

    // The COOLER path through this guard is not re-tested here: it is pinned
    // end-to-end by `cooler_reference.rs::cooling_below_absolute_zero_is_rejected`,
    // through a real scenario and solver. A copy at this level could only fail
    // together with that one, so it would add a maintenance point and no
    // discrimination. What is genuinely new is the shared checker above.

    // -----------------------------------------------------------------------
    // Heat exchanger
    // -----------------------------------------------------------------------

    /// Two independent streams, coupled. `flows` are hand-built as everywhere
    /// else in this module: the exchanger's contract is thermal, and routing it
    /// through a hydraulic solve would only add a way for the test to fail for
    /// an unrelated reason.
    ///
    /// The hot side is deliberately the SMALLER stream (1 kg/s against 3), so
    /// `C_a ≠ C_b` and `C_min` is distinguishable from `C_max`. With equal
    /// capacity rates the two are the same number and the choice is untestable.
    fn coupled_pair(
        hot_inlet: Kelvin,
        cold_inlet: Kelvin,
        effectiveness: f64,
    ) -> (PlantGraph, BTreeMap<EdgeId, f64>, NodeId, NodeId) {
        let mut g = PlantGraph::new();
        let hot_src = g.add_node(source("hot_src", hot_inlet));
        let hot_side = g.add_node(node("hot_side", NodeKind::HeatExchanger));
        let hot_out = g.add_node(node(
            "hot_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let cold_src = g.add_node(source("cold_src", cold_inlet));
        let cold_side = g.add_node(node("cold_side", NodeKind::HeatExchanger));
        let cold_out = g.add_node(node(
            "cold_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));

        let hot_in = g.add_pipe(hot_src, hot_side, pipe("hot_in"));
        let hot_o = g.add_pipe(hot_side, hot_out, pipe("hot_out"));
        let cold_in = g.add_pipe(cold_src, cold_side, pipe("cold_in"));
        let cold_o = g.add_pipe(cold_side, cold_out, pipe("cold_out"));

        g.add_coupling(HeatExchangerCoupling {
            side_a: hot_side,
            side_b: cold_side,
            effectiveness,
        });

        let flows = BTreeMap::from([(hot_in, 1.0), (hot_o, 1.0), (cold_in, 3.0), (cold_o, 3.0)]);
        (g, flows, hot_side, cold_side)
    }

    /// Hand calculation, ΔT-effectiveness (DESIGN §4a):
    ///
    /// ```text
    /// C_hot  = 1·4184 = 4184 W/K      C_cold = 3·4184 = 12552 W/K
    /// C_min  = 4184 W/K               ΔT_in  = 400 − 300 = 100 K
    /// Q      = 0.5 · 4184 · 100 = 209 200 W
    /// T_hot_out  = 400 − 209200/4184  = 350 K exactly
    /// T_cold_out = 300 + 209200/12552 = 300 + 50/3 K
    /// ```
    ///
    /// The asymmetry is the point: the same Q moves both outlets, but by
    /// different amounts, so the test would fail if either side used the wrong
    /// capacity rate. Using `C_max` instead of `C_min` would give Q = 627 600 W
    /// and cool the hot stream to 250 K — BELOW the cold inlet, which is the
    /// second-law violation `C_min` exists to prevent.
    #[test]
    fn an_exchanger_transfers_effectiveness_times_c_min() {
        let (g, flows, hot_side, cold_side) = coupled_pair(Kelvin(400.0), Kelvin(300.0), 0.5);
        let temperature = resolve(&g, &flows).expect("two independent streams must resolve");

        assert!(
            (temperature[&hot_side].value() - 350.0).abs() < 1e-9,
            "hot outlet must be 350 K, got {}",
            temperature[&hot_side].value()
        );
        assert!(
            (temperature[&cold_side].value() - (300.0 + 50.0 / 3.0)).abs() < 1e-9,
            "cold outlet must be 300 + 50/3 K, got {}",
            temperature[&cold_side].value()
        );

        // The duty leaving one stream is the duty entering the other, to the
        // last bit the floats allow: one signed Q, applied twice (DESIGN §4a).
        let given = 4184.0 * (400.0 - temperature[&hot_side].value());
        let taken = 3.0 * 4184.0 * (temperature[&cold_side].value() - 300.0);
        assert!(
            (given - taken).abs() < 1e-6,
            "energy must balance across the exchanger: {given} W out, {taken} W in"
        );
    }

    /// Neither side is hardcoded as the hot one. With the inlets swapped, the
    /// SAME plant must run the heat the other way — the sign of
    /// `T_a_in − T_b_in` is the only thing that decides direction.
    ///
    /// Mirrors the reference above: the small stream now GAINS 50 K and the
    /// large one loses 50/3 K.
    #[test]
    fn heat_flows_from_whichever_side_is_hotter() {
        let (g, flows, side_a, side_b) = coupled_pair(Kelvin(300.0), Kelvin(400.0), 0.5);
        let temperature = resolve(&g, &flows).expect("two independent streams must resolve");

        assert!(
            (temperature[&side_a].value() - 350.0).abs() < 1e-9,
            "the colder small stream must be HEATED to 350 K, got {}",
            temperature[&side_a].value()
        );
        assert!(
            (temperature[&side_b].value() - (400.0 - 50.0 / 3.0)).abs() < 1e-9,
            "the hotter large stream must be COOLED to 400 − 50/3 K, got {}",
            temperature[&side_b].value()
        );
    }

    /// ε scales the duty linearly, and ε = 1 is the thermodynamic limit: the
    /// small stream leaves at exactly the other inlet's temperature, never past
    /// it. This is the boundary `C_min` guarantees and `C_max` would breach.
    #[test]
    fn full_effectiveness_approaches_the_other_inlet_without_crossing_it() {
        let (g, flows, hot_side, cold_side) = coupled_pair(Kelvin(400.0), Kelvin(300.0), 1.0);
        let temperature = resolve(&g, &flows).expect("two independent streams must resolve");

        assert!(
            (temperature[&hot_side].value() - 300.0).abs() < 1e-9,
            "at ε = 1 the C_min stream must reach the other inlet exactly, got {}",
            temperature[&hot_side].value()
        );
        // The second-law bound is each outlet against the OTHER STREAM'S INLET,
        // not against the other outlet. A cold outlet above the hot outlet is
        // ordinary counter-current behaviour, not a violation — and this model
        // draws no co-/counter-current distinction, so asserting the outlets
        // stay ordered would pin a restriction the physics does not impose.
        assert!(
            temperature[&hot_side].value() >= 300.0 - 1e-9,
            "the hot stream must not be cooled below the cold inlet, got {}",
            temperature[&hot_side].value()
        );
        assert!(
            temperature[&cold_side].value() <= 400.0 + 1e-9,
            "the cold stream must not be heated above the hot inlet, got {}",
            temperature[&cold_side].value()
        );
    }

    /// The pair is ONE vertex in the sweep, and this is the test that says so.
    ///
    /// The cold side is fed through a junction, so its inlet is not known until
    /// that junction resolves — while the hot side's own inflow is ready
    /// immediately and carries the lower node id. A per-node sweep therefore
    /// reaches the hot side FIRST and has to read a cold inlet that does not
    /// exist yet. Merged, the pair waits for the union of both sides'
    /// dependencies, which is exactly when both inlets are known.
    #[test]
    fn an_exchanger_pair_waits_for_both_sides_upstreams() {
        let mut g = PlantGraph::new();
        let hot_src = g.add_node(source("hot_src", Kelvin(400.0)));
        let hot_side = g.add_node(node("hot_side", NodeKind::HeatExchanger));
        let hot_out = g.add_node(node(
            "hot_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let cold_src = g.add_node(source("cold_src", Kelvin(300.0)));
        let cold_side = g.add_node(node("cold_side", NodeKind::HeatExchanger));
        let cold_out = g.add_node(node(
            "cold_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        // Higher node id than either side, so the sweep pops it LAST if it is
        // ordering by node rather than by vertex.
        let mid = g.add_node(node("mid", NodeKind::Junction));

        let hot_in = g.add_pipe(hot_src, hot_side, pipe("hot_in"));
        let hot_o = g.add_pipe(hot_side, hot_out, pipe("hot_out"));
        let cold_feed = g.add_pipe(cold_src, mid, pipe("cold_feed"));
        let cold_in = g.add_pipe(mid, cold_side, pipe("cold_in"));
        let cold_o = g.add_pipe(cold_side, cold_out, pipe("cold_out"));

        g.add_coupling(HeatExchangerCoupling {
            side_a: hot_side,
            side_b: cold_side,
            effectiveness: 0.5,
        });

        let flows = BTreeMap::from([
            (hot_in, 1.0),
            (hot_o, 1.0),
            (cold_feed, 3.0),
            (cold_in, 3.0),
            (cold_o, 3.0),
        ]);
        let temperature = resolve(&g, &flows).expect("the pair must wait for the junction");

        // Same hand calculation as the reference: the junction is a pure
        // pass-through, so the numbers must be untouched by its presence.
        assert!(
            (temperature[&hot_side].value() - 350.0).abs() < 1e-9,
            "hot outlet must still be 350 K, got {}",
            temperature[&hot_side].value()
        );
        assert!(
            (temperature[&cold_side].value() - (300.0 + 50.0 / 3.0)).abs() < 1e-9,
            "cold outlet must still be 300 + 50/3 K, got {}",
            temperature[&cold_side].value()
        );
    }

    /// One side feeding the other is a genuine circular definition — side A's
    /// outlet depends on B's inlet, which IS A's outlet — so it must land in
    /// the existing recycle rejection rather than resolve against a stale
    /// value. This is why the merge skips self-dependencies in BOTH the count
    /// and the release: discounting them in only one place would make this
    /// plant silently ready.
    #[test]
    fn an_exchanger_feeding_its_own_partner_is_rejected() {
        let mut g = PlantGraph::new();
        let src = g.add_node(source("src", Kelvin(400.0)));
        let side_a = g.add_node(node("side_a", NodeKind::HeatExchanger));
        let side_b = g.add_node(node("side_b", NodeKind::HeatExchanger));
        let out = g.add_node(node(
            "out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let feed = g.add_pipe(src, side_a, pipe("feed"));
        let across = g.add_pipe(side_a, side_b, pipe("across"));
        let drain = g.add_pipe(side_b, out, pipe("drain"));

        g.add_coupling(HeatExchangerCoupling {
            side_a,
            side_b,
            effectiveness: 0.5,
        });

        let flows = BTreeMap::from([(feed, 1.0), (across, 1.0), (drain, 1.0)]);
        let err = resolve(&g, &flows)
            .expect_err("an exchanger in series with itself is mutually defined");
        let message = err.to_string();
        assert!(
            message.contains("recycle"),
            "the error must explain what went wrong, got: {message}"
        );
        for name in ["side_a", "side_b"] {
            assert!(
                message.contains(name),
                "the error must name the stuck side {name}, got: {message}"
            );
        }
    }

    /// An exchanger with one stream stopped is a pipe: no throughput on a side
    /// means no capacity rate to transfer against, so the duty is zero and the
    /// running side passes its inlet straight through. Physics, not a guard —
    /// but worth pinning, because the alternative (dividing by a zero capacity)
    /// produces NaN that would propagate as an ordinary temperature.
    #[test]
    fn a_stalled_side_transfers_nothing() {
        let (mut g, mut flows, hot_side, cold_side) =
            coupled_pair(Kelvin(400.0), Kelvin(300.0), 0.5);
        let _ = &mut g;
        for flow in flows.values_mut() {
            // Stop the cold stream only; its two pipes carry 3.0.
            if *flow == 3.0 {
                *flow = 0.0;
            }
        }
        let temperature = resolve(&g, &flows).expect("a stalled side must not break the sweep");

        assert!(
            (temperature[&hot_side].value() - 400.0).abs() < 1e-12,
            "with nothing to exchange with, the hot side must pass 400 K through, got {}",
            temperature[&hot_side].value()
        );
        assert!(
            temperature[&cold_side].value().is_finite(),
            "the stalled side must hold a finite temperature, got {}",
            temperature[&cold_side].value()
        );
    }

    /// A junction nothing flows through is indeterminate (0/0), not broken. It
    /// must hold the last value it saw — finite and reproducible — because mass
    /// balance means it carries no enthalpy anywhere regardless.
    #[test]
    fn a_stagnant_junction_holds_its_previous_temperature() {
        let mut g = PlantGraph::new();
        let src = g.add_node(source("src", Kelvin(280.0)));
        let idle = g.add_node(node("idle", NodeKind::Junction));
        let dead = g.add_pipe(src, idle, pipe("dead_leg"));

        let flows = BTreeMap::from([(dead, 0.0)]);
        let previous = NodeStates {
            temperature: BTreeMap::from([(idle, Kelvin(311.0))]),
            composition: BTreeMap::new(),
            ..Default::default()
        };
        let temperature = resolve_node_states(
            &g,
            &Slate::water_only(),
            &flows,
            &no_friction(),
            &NoRxn,
            &NoSeparation,
            &TestThermo,
            &ConstantEnthalpyStub,
            &NoLineFlashStub,
            &everywhere_atmospheric(&g),
            &previous,
            &BTreeMap::new(),
            Seconds(1.0),
        )
        .unwrap()
        .temperature;
        assert_eq!(
            temperature[&idle].value(),
            311.0,
            "must hold the last value"
        );

        // With no history at all it falls back to ambient rather than NaN.
        let fresh = resolve(&g, &flows).unwrap();
        assert_eq!(fresh[&idle].value(), T_AMBIENT.value());
    }

    // -----------------------------------------------------------------------
    // The pipe ambient transform, as a pure function.
    // -----------------------------------------------------------------------
    //
    // Tested here rather than only through an engine because the closed form is
    // worth pinning against a HAND-COMPUTED number, and an engine-level test
    // cannot do that: the mass flow comes out of the solver, so the expected
    // value would have to be built by re-evaluating the same formula the code
    // just evaluated — a tautology that passes whatever the exponent's sign is.
    // The engine-level file (`solvers/tests/pipe_ambient_reference.rs`) tests
    // the complementary thing: that this function is actually WIRED IN, and it
    // cross-checks the physics against the log-mean form, which is independent.
    mod pipe_ambient {
        use super::*;

        /// Water's cp [J/(kg·K)] — written out rather than read from the slate,
        /// so both sides of the reference do not share one source of truth.
        const CP: JPerKgK = JPerKgK(4184.0);

        /// REFERENCE — hand computation at exactly one transfer unit.
        ///
        /// Chosen so `UA/(ṁ·cp)` is exactly 1 and the decay is `e⁻¹`, a constant
        /// that can be written down independently of the code:
        ///
        ///   ṁ·cp   = 1 kg/s · 4184 J/(kg·K) = 4184 W/K
        ///   UA     = 4184 W/K            ⇒ exponent = −1
        ///   e⁻¹    = 0.367879441171442334
        ///   ΔT_in  = 373.15 − 293.15 = 80 K
        ///   T_out  = 293.15 + 80·e⁻¹ = 293.15 + 29.4303552937153867
        ///          = 322.580355293715387 K
        ///
        /// A wrong-signed exponent gives 293.15 + 80·e = 510.6 K, and an Euler
        /// step gives 293.15 + 80·(1−1) = 293.15 K exactly — both far outside
        /// the tolerance, which is round-off because this form is analytic and
        /// has no truncation error to budget for (contrast the tank's 1e-3).
        #[test]
        fn one_transfer_unit_decays_by_exactly_e_inverse() {
            let out = pipe_outlet_temperature(
                Kelvin(373.15),
                WattPerKelvin(4184.0),
                KgPerSec(1.0),
                CP,
                Watt::ZERO,
            );
            // The hand computation above gives 322.580355293715387; this is
            // that value truncated to what an f64 can actually hold. The
            // difference is ~1e-13, well inside the 1e-9 tolerance, so the
            // reference is still the hand calculation and not a readback.
            let expected = 322.580_355_293_715_4;
            assert!(
                (out.value() - expected).abs() < 1e-9,
                "one transfer unit must decay by e⁻¹ to {expected} K, got {}",
                out.value()
            );
        }

        /// `UA = 0` is the identity, which is what makes every scenario written
        /// before this field existed bit-identical. Asserted EXACTLY: `exp(0)`
        /// is 1.0 to the bit, so any tolerance here would hide a near-miss.
        #[test]
        fn no_ua_is_exactly_the_identity() {
            let inlet = Kelvin(373.15);
            let out =
                pipe_outlet_temperature(inlet, WattPerKelvin::ZERO, KgPerSec(2.5), CP, Watt::ZERO);
            assert_eq!(out.value(), inlet.value());
        }

        /// The `|ṁ|` requirement. Reversing the flow reverses which end is the
        /// inlet — a decision made before this function is called — so the
        /// transform itself must be blind to the sign.
        ///
        /// Under a signed `ṁ` the exponent flips and this returns 510.6 K: the
        /// pipe would run AWAY from ambient, heating an already-hot stream. That
        /// is the mutation this test exists to catch, and it catches it here
        /// rather than in a plant, so the coverage does not depend on some
        /// scenario happening to reverse a flow.
        #[test]
        fn reverse_flow_decays_identically() {
            let forward = pipe_outlet_temperature(
                Kelvin(373.15),
                WattPerKelvin(4184.0),
                KgPerSec(1.0),
                CP,
                Watt::ZERO,
            );
            let reverse = pipe_outlet_temperature(
                Kelvin(373.15),
                WattPerKelvin(4184.0),
                KgPerSec(-1.0),
                CP,
                Watt::ZERO,
            );
            assert_eq!(
                forward.value(),
                reverse.value(),
                "the transform must depend on |ṁ| only — the flow's sign already \
                 picked the inlet end"
            );
        }

        /// The reason for analytic rather than Euler, stated as a test.
        ///
        /// At 100 transfer units an explicit step, `T + (UA/(ṁ·cp))·(T_amb − T)`,
        /// would land at 293.15 − 99·80 ≈ −7627 K: past ambient, past absolute
        /// zero, and diverging further every tick. `exp` cannot cross ambient at
        /// any exponent, so the outlet is bracketed by ambient below and the
        /// inlet above no matter how extreme the ratio.
        #[test]
        fn a_huge_transfer_ratio_approaches_ambient_but_never_crosses_it() {
            let out = pipe_outlet_temperature(
                Kelvin(373.15),
                WattPerKelvin(418_400.0), // 100 transfer units
                KgPerSec(1.0),
                CP,
                Watt::ZERO,
            );
            assert!(
                out.value() >= T_AMBIENT.value() && out.value() <= 373.15,
                "the outlet must stay between ambient and the inlet, got {}",
                out.value()
            );
            assert!(
                (out.value() - T_AMBIENT.value()).abs() < 1e-9,
                "100 transfer units must land on ambient, got {}",
                out.value()
            );
        }

        /// The zero-flow guard, on the case that actually reaches it.
        ///
        /// `ṁ = 0` with `UA = 0` is `0/0`. Without the guard this returns NaN,
        /// which no downstream check attributes to a pipe — it surfaces as a
        /// bare non-finite. This is the DEFAULT pipe with a closed valve, so it
        /// is the reachable case, not a contrived one.
        #[test]
        fn a_stagnant_uninsulated_pipe_is_finite_not_nan() {
            let out = pipe_outlet_temperature(
                Kelvin(373.15),
                WattPerKelvin::ZERO,
                KgPerSec::ZERO,
                CP,
                Watt::ZERO,
            );
            assert!(
                out.value().is_finite(),
                "a stagnant pipe must not produce a NaN temperature, got {}",
                out.value()
            );
            assert_eq!(out.value(), 373.15);
        }

        /// The other stagnant case: zero flow with a real `UA`. This one is
        /// finite even without the guard (`−∞ → exp = 0 → ambient`), so it is
        /// documenting a CHOICE rather than preventing a NaN. Holding the inlet
        /// is the honest answer at this fidelity: with no throughput there is no
        /// stream to carry the heat, and a pipe that instantly equilibrated with
        /// ambient the moment its valve shut would be modelling a wall thermal
        /// mass the engine does not have.
        #[test]
        fn a_stagnant_insulated_pipe_holds_its_inlet_rather_than_snapping_to_ambient() {
            let out = pipe_outlet_temperature(
                Kelvin(373.15),
                WattPerKelvin(4184.0),
                KgPerSec::ZERO,
                CP,
                Watt::ZERO,
            );
            assert_eq!(
                out.value(),
                373.15,
                "a stagnant pipe holds its inlet; snapping to ambient would claim \
                 a wall thermal mass this fidelity does not model"
            );
        }

        /// Ambient exchange is not one-directional. A stream COLDER than ambient
        /// must warm toward it through the same term, with no second code path —
        /// the same property `ambient_exchange` has for tanks, and reachable in
        /// a plant with a `Cooler` upstream.
        #[test]
        fn a_cold_stream_warms_toward_ambient() {
            let out = pipe_outlet_temperature(
                Kelvin(273.15),
                WattPerKelvin(4184.0),
                KgPerSec(1.0),
                CP,
                Watt::ZERO,
            );
            assert!(
                out.value() > 273.15 && out.value() < T_AMBIENT.value(),
                "a stream below ambient must warm toward it, got {}",
                out.value()
            );
        }
    }

    /// The reactor's contract in the sweep — imposed `t_set`, product composition,
    /// and the two emergent duties — with hand-built flows and a KNOWN converting
    /// reaction, so the numbers are all traceable to arithmetic.
    mod reactor_tests {
        use super::*;
        use crate::components::{Phase, PseudoComponent};

        /// A two-cut slate whose components have DELIBERATELY different `cp`, so a
        /// composition change shifts the mixture `cp` — the property the reactor's
        /// sensible duty must capture (`cp_out` from the products, not the feed).
        fn two_cut_slate() -> Slate {
            let cut = |name: &str, cp: f64| PseudoComponent {
                name: name.into(),
                tb: Kelvin(400.0),
                molar_mass: KgPerMol(0.1),
                density: Some(KgPerM3(800.0)),
                cp: JPerKgK(cp),
                cp_shape: None,
                phase: Phase::Liquid,
                dh_vap: None,
                cp_vapour: None,
                free_water: false,
            };
            Slate::new(vec![cut("feed_lump", 2000.0), cut("product_lump", 3000.0)]).unwrap()
        }

        /// Converts ANY feed into a fixed product slate with a fixed heat of
        /// reaction. The lookup fidelity's job in miniature: the numbers here are
        /// what the energy gate recomputes against, independently.
        struct FixedConversion {
            products: Composition,
            dh_rxn: JPerKg,
        }
        impl ReactionModel for FixedConversion {
            fn name(&self) -> &'static str {
                "test-fixed"
            }
            fn react(
                &self,
                _feed: &Composition,
                _t: Kelvin,
                _tau: Seconds,
                _slate: &Slate,
            ) -> Result<Reaction, SimError> {
                Ok(Reaction {
                    products: self.products.clone(),
                    dh_rxn: self.dh_rxn,
                })
            }
        }

        /// source → reactor → sink, one reactor pass. Asserts the two duties
        /// SEPARATELY against an independent recompute: the sensible term against
        /// `ṁ·(cp_out·(T_set−T_REF) − cp_in·(T_in−T_REF))`, and the reported term
        /// as the DIFFERENCE `reported − sensible == ṁ·Δh_rxn` — never the reported
        /// duty against its own formula, which would be a tautology. So `Δh_rxn`
        /// and the cp-shift each falsify their own term (verified by mutation:
        /// swapping `cp_out`→`cp_in` breaks the sensible assert; a wrong `Δh_rxn`
        /// breaks the difference).
        #[test]
        fn the_two_duties_pin_the_cp_shift_and_the_heat_of_reaction_separately() {
            let slate = two_cut_slate();
            let t_in = Kelvin(300.0);
            let t_set = Kelvin(800.0);
            let mdot = 5.0;
            let dh_rxn = JPerKg(1.0e5); // endothermic

            // Feed is pure lump 0 (cp 2000); products 40/60 (cp 2600).
            let feed = Composition::pure(2, 0);
            let products = Composition::from_weights(&[0.4, 0.6]).unwrap();

            let mut g = PlantGraph::new();
            let src = g.add_node(node(
                "src",
                NodeKind::Source {
                    pressure: Pascal(2.0e5),
                    temperature: t_in,
                    composition: feed.clone(),
                },
            ));
            let rx = g.add_node(node(
                "rx",
                NodeKind::Reactor {
                    t_set,
                    tau: Seconds(2.0),
                },
            ));
            let snk = g.add_node(node(
                "snk",
                NodeKind::Sink {
                    pressure: Pascal(1.0e5),
                    temperature: t_in,
                    composition: feed.clone(),
                },
            ));
            let feed_edge = g.add_pipe(src, rx, pipe("feed"));
            let prod_edge = g.add_pipe(rx, snk, pipe("prod"));
            let flows = BTreeMap::from([(feed_edge, mdot), (prod_edge, mdot)]);

            let reactions = FixedConversion {
                products: products.clone(),
                dh_rxn,
            };
            let states = resolve_node_states(
                &g,
                &slate,
                &flows,
                &no_friction(),
                &reactions,
                &NoSeparation,
                &TestThermo,
                &ConstantEnthalpyStub,
                &NoLineFlashStub,
                &everywhere_atmospheric(&g),
                &NodeStates::default(),
                &BTreeMap::new(),
                Seconds(1.0),
            )
            .unwrap();

            // The reactor imposes t_set and carries the PRODUCTS downstream.
            assert_eq!(states.temperature[&rx], t_set);
            assert_eq!(states.composition[&rx].fractions(), products.fractions());

            let duty = states.reactor_duty[&rx];

            // Independent recompute of the sensible baseline — cp_out from the
            // PRODUCTS, cp_in from the FEED, not the engine's own value.
            let cp_in = feed.mixture_cp(&slate).value();
            let cp_out = products.mixture_cp(&slate).value();
            let tref = T_REF.value();
            let sensible_expected =
                mdot * (cp_out * (t_set.value() - tref) - cp_in * (t_in.value() - tref));

            assert!(
                (duty.sensible.value() - sensible_expected).abs() < 1e-6,
                "emergent sensible duty {} W must equal the independent baseline {} W \
                 (this is the cp-shift term)",
                duty.sensible.value(),
                sensible_expected
            );
            // The reported duty sits exactly ṁ·Δh_rxn above the sensible baseline.
            let difference = duty.reported.value() - sensible_expected;
            assert!(
                (difference - mdot * dh_rxn.value()).abs() < 1e-6,
                "reported − sensible = {} W must equal ṁ·Δh_rxn = {} W (the heat-of-\
                 reaction term)",
                difference,
                mdot * dh_rxn.value()
            );
        }

        /// A reactor with no throughput still resolves — `t_set` on the outlet, the
        /// reaction run on the (inert) feed, and BOTH duties zero because ṁ = 0.
        /// The divide-by-capacity is never reached.
        #[test]
        fn a_dead_reactor_holds_its_setpoint_with_zero_duty() {
            let slate = two_cut_slate();
            let products = Composition::from_weights(&[0.4, 0.6]).unwrap();

            let mut g = PlantGraph::new();
            let src = g.add_node(node(
                "src",
                NodeKind::Source {
                    pressure: Pascal(2.0e5),
                    temperature: Kelvin(300.0),
                    composition: Composition::pure(2, 0),
                },
            ));
            let rx = g.add_node(node(
                "rx",
                NodeKind::Reactor {
                    t_set: Kelvin(800.0),
                    tau: Seconds(2.0),
                },
            ));
            let snk = g.add_node(node(
                "snk",
                NodeKind::Sink {
                    pressure: Pascal(1.0e5),
                    temperature: Kelvin(300.0),
                    composition: Composition::pure(2, 0),
                },
            ));
            let feed_edge = g.add_pipe(src, rx, pipe("feed"));
            let prod_edge = g.add_pipe(rx, snk, pipe("prod"));
            // No flow anywhere.
            let flows = BTreeMap::from([(feed_edge, 0.0), (prod_edge, 0.0)]);

            let reactions = FixedConversion {
                products,
                dh_rxn: JPerKg(1.0e5),
            };
            let states = resolve_node_states(
                &g,
                &slate,
                &flows,
                &no_friction(),
                &reactions,
                &NoSeparation,
                &TestThermo,
                &ConstantEnthalpyStub,
                &NoLineFlashStub,
                &everywhere_atmospheric(&g),
                &NodeStates::default(),
                &BTreeMap::new(),
                Seconds(1.0),
            )
            .unwrap();

            assert_eq!(states.temperature[&rx], Kelvin(800.0));
            let duty = states.reactor_duty[&rx];
            assert_eq!(duty.sensible.value(), 0.0);
            assert_eq!(duty.reported.value(), 0.0);
        }
    }

    /// The catalyst bed's contract in the sweep (M57.1, docs/DESIGN.md §62.1–2):
    /// its books, its lag, its residence time, its zero flow, and a
    /// heat-absorbing reaction settling below its feed. Each test drives the
    /// sweep tick by tick and writes the bed's end-of-tick temperature back as
    /// `Engine::tick` does, so what it pins is the step the engine runs.
    mod bed_tests {
        use super::*;
        use crate::graph::{BedWall, CatalystBed, WallState};

        const FEED_CP: f64 = 2000.0;
        const PRODUCT_CP: f64 = 3000.0;
        const FEED_DENSITY: f64 = 800.0;
        const PRODUCT_DENSITY: f64 = 600.0;

        /// Two liquid cuts with DIFFERENT `cp` and density, so a conversion
        /// shifts the products' capacity (the books gate) and a mixed feed's
        /// density is a harmonic mean rather than either one (the `τ` gate).
        fn two_cut_slate() -> Slate {
            use crate::components::{Phase, PseudoComponent};
            let cut = |name: &str, cp: f64, density: f64| PseudoComponent {
                name: name.into(),
                tb: Kelvin(400.0),
                molar_mass: KgPerMol(0.1),
                density: Some(KgPerM3(density)),
                cp: JPerKgK(cp),
                cp_shape: None,
                phase: Phase::Liquid,
                dh_vap: None,
                cp_vapour: None,
                free_water: false,
            };
            Slate::new(vec![
                cut("feed_lump", FEED_CP, FEED_DENSITY),
                cut("product_lump", PRODUCT_CP, PRODUCT_DENSITY),
            ])
            .unwrap()
        }

        /// First-order `A → B`, Arrhenius in the bed's temperature, solved
        /// exactly over `τ` (`+∞` included): `X = 1 − exp(−k(T)·τ)`, and
        /// `Δh_rxn = f_A·X·h` per kg of feed. A test model, written here so the
        /// bed's gates do not wait on M57.2's chemistry.
        struct FirstOrder {
            /// Rate constant at `t_ref` [1/s].
            k_ref: f64,
            t_ref: f64,
            /// `E/R` [K].
            e_over_r: f64,
            /// Heat of reaction per kg CONVERTED [J/kg]; positive absorbs.
            h: f64,
        }
        impl FirstOrder {
            fn conversion(&self, temperature: f64, tau: f64) -> f64 {
                let k =
                    self.k_ref * (-self.e_over_r * (1.0 / temperature - 1.0 / self.t_ref)).exp();
                -(-k * tau).exp_m1()
            }
        }
        impl ReactionModel for FirstOrder {
            fn name(&self) -> &'static str {
                "test-first-order"
            }
            fn react(
                &self,
                feed: &Composition,
                temperature: Kelvin,
                tau: Seconds,
                _slate: &Slate,
            ) -> Result<Reaction, SimError> {
                let x = self.conversion(temperature.value(), tau.value());
                let [a, b] = [feed.fractions()[0], feed.fractions()[1]];
                Ok(Reaction {
                    products: Composition::from_weights(&[a * (1.0 - x), b + a * x])?,
                    dh_rxn: JPerKg(a * x * self.h),
                })
            }
        }

        /// source → bed → sink, the bed loaded at `start` with heat capacity
        /// `capacity` [J/K] and catalyst `volume` [m³].
        fn bed_plant(
            feed: &Composition,
            t_in: Kelvin,
            start: Kelvin,
            capacity: f64,
            volume: f64,
        ) -> (PlantGraph, NodeId, EdgeId, EdgeId) {
            let mut g = PlantGraph::new();
            let src = g.add_node(node(
                "src",
                NodeKind::Source {
                    pressure: Pascal(2.0e5),
                    temperature: t_in,
                    composition: feed.clone(),
                },
            ));
            let bed = g.add_node(node(
                "bed",
                NodeKind::BedReactor {
                    bed: CatalystBed {
                        heat_capacity: JPerK(capacity),
                        catalyst_volume: CubicMeter(volume),
                        temperature: start,
                    },
                    // Never fails: no gate here is about the wall (M58).
                    wall: BedWall {
                        failure_temperature: Kelvin(1.0e6),
                        rupture_area: SquareMeter(1.0e-4),
                        heating_value: JPerKg(0.0),
                        fire_share: 0.0,
                        hole: None,
                        hydrogen_leak: None,
                        state: WallState::Intact,
                    },
                },
            ));
            let snk = g.add_node(node(
                "snk",
                NodeKind::Sink {
                    pressure: Pascal(1.0e5),
                    temperature: t_in,
                    composition: feed.clone(),
                },
            ));
            let feed_edge = g.add_pipe(src, bed, pipe("feed"));
            let prod_edge = g.add_pipe(bed, snk, pipe("prod"));
            (g, bed, feed_edge, prod_edge)
        }

        fn flows(feed: EdgeId, prod: EdgeId, mass_flow: f64) -> BTreeMap<EdgeId, f64> {
            BTreeMap::from([(feed, mass_flow), (prod, mass_flow)])
        }

        fn bed_temperature(g: &PlantGraph, bed: NodeId) -> Kelvin {
            match &g.node(bed).kind {
                NodeKind::BedReactor { bed, .. } => bed.temperature,
                _ => unreachable!("the fixture's bed is a bed"),
            }
        }

        /// One tick: the sweep, then the write-back `Engine::tick` does.
        fn step(
            g: &mut PlantGraph,
            slate: &Slate,
            flows: &BTreeMap<EdgeId, f64>,
            reactions: &dyn ReactionModel,
            previous: &NodeStates,
            dt: f64,
        ) -> NodeStates {
            let states = resolve_node_states(
                g,
                slate,
                flows,
                &no_friction(),
                reactions,
                &NoSeparation,
                &TestThermo,
                &ConstantEnthalpyStub,
                &NoLineFlashStub,
                &everywhere_atmospheric(g),
                previous,
                &BTreeMap::new(),
                Seconds(dt),
            )
            .unwrap();
            for (&id, pass) in &states.bed {
                if let NodeKind::BedReactor { bed, .. } = &mut g.node_mut(id).kind {
                    bed.temperature = pass.temperature;
                }
            }
            states
        }

        /// **The first law at the bed closes**: what the bed stored equals what
        /// came in, plus what the reaction released, less what left — with the
        /// outflow recomputed BY HAND from the reported outlet temperature on
        /// the PRODUCTS' capacity, and the released heat recomputed by hand at
        /// the bed's START-of-tick temperature. A conversion that shifts `cp`
        /// is what makes the products' capacity matter.
        #[test]
        fn the_bed_books_close_exactly() {
            let slate = two_cut_slate();
            let feed = Composition::pure(2, 0);
            let (t_in, start, capacity, volume, mass_flow, dt) =
                (Kelvin(600.0), Kelvin(650.0), 4.0e5, 1.0, 2.0, 1.0);
            let reactions = FirstOrder {
                k_ref: 0.002,
                t_ref: 650.0,
                e_over_r: 15_000.0,
                h: -3.0e5,
            };
            let (mut g, bed, fe, pe) = bed_plant(&feed, t_in, start, capacity, volume);
            let states = step(
                &mut g,
                &slate,
                &flows(fe, pe, mass_flow),
                &reactions,
                &NodeStates::default(),
                dt,
            );

            let pass = states.bed[&bed];
            let tau = volume * FEED_DENSITY / mass_flow;
            let x = reactions.conversion(start.value(), tau);
            assert!(x > 0.1, "the fixture must convert something (X = {x})");
            let released = -mass_flow * x * reactions.h;
            assert!(
                (pass.reaction_heat.value() - released).abs() <= 1e-9 * released.abs(),
                "released {} W, by hand at the START temperature {released} W",
                pass.reaction_heat.value()
            );

            let tref = T_REF.value();
            let heat_in = mass_flow * FEED_CP * (t_in.value() - tref);
            let product_cp = (1.0 - x) * FEED_CP + x * PRODUCT_CP;
            let outlet = states.temperature[&bed].value();
            let heat_out = mass_flow * product_cp * (outlet - tref);
            let stored = capacity * (pass.temperature.value() - start.value()) / dt;
            let imbalance = heat_in + released - heat_out - stored;
            assert!(
                imbalance.abs() <= 1e-9 * heat_in.abs(),
                "the bed's first law misses by {imbalance} W (in {heat_in}, released \
                 {released}, out {heat_out}, stored {stored})"
            );
            // The products travel on, and the fluid leaves near the bed.
            assert!((states.composition[&bed].fractions()[1] - x).abs() < 1e-12);
            assert!(!states.held.contains(&bed));
        }

        /// **No reaction, one cut: the lumped-capacitance lag of a hand
        /// calculation**, `T_b(t) = T_in + (T₀ − T_in)·e^(−W·t/C)` (Incropera &
        /// DeWitt 6th ed. §5.1), which the step reproduces to rounding at every
        /// tick because, with nothing frozen that moves, it is exact.
        #[test]
        fn a_bed_with_no_reaction_lags_its_feed_as_a_lumped_capacitance() {
            let slate = two_cut_slate();
            let feed = Composition::pure(2, 0);
            let (t_in, start, capacity, mass_flow, dt) = (300.0, 500.0, 4.0e5, 2.0, 1.0);
            let (mut g, bed, fe, pe) = bed_plant(&feed, Kelvin(t_in), Kelvin(start), capacity, 1.0);
            let flows = flows(fe, pe, mass_flow);
            let rate = mass_flow * FEED_CP / capacity; // W/C = 1/(100 s)
            let mut previous = NodeStates::default();
            for n in 1..=300 {
                previous = step(&mut g, &slate, &flows, &NoRxn, &previous, dt);
                let expected = t_in + (start - t_in) * (-rate * dt * f64::from(n)).exp();
                let got = bed_temperature(&g, bed).value();
                assert!(
                    (got - expected).abs() <= 1e-9 * (start - t_in),
                    "tick {n}: bed {got} K, hand {expected} K"
                );
            }
            // Three time constants on, the fluid follows the bed down.
            let fluid = previous.temperature[&bed].value();
            assert!(fluid < t_in + (start - t_in) * 0.06 && fluid > t_in);
        }

        /// **`τ = V·ρ_feed/ṁ` by hand**, with `ρ_feed` the mass-weighted harmonic
        /// mean of the declared densities, and it follows the flow: half the feed,
        /// twice the time.
        #[test]
        fn the_residence_time_follows_the_flow() {
            let slate = two_cut_slate();
            let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
            let volume = 2.0;
            let (mut g, bed, fe, pe) =
                bed_plant(&feed, Kelvin(500.0), Kelvin(500.0), 4.0e5, volume);
            let density = 1.0 / (0.5 / FEED_DENSITY + 0.5 / PRODUCT_DENSITY);
            for mass_flow in [4.0, 2.0] {
                let states = step(
                    &mut g,
                    &slate,
                    &flows(fe, pe, mass_flow),
                    &NoRxn,
                    &NodeStates::default(),
                    1.0,
                );
                let tau = states.bed[&bed]
                    .residence_time
                    .expect("a flowing bed has a residence time")
                    .value();
                let expected = volume * density / mass_flow;
                assert!(
                    (tau - expected).abs() <= 1e-12 * expected,
                    "τ = {tau} s at {mass_flow} kg/s, by hand {expected} s"
                );
            }
        }

        /// **Zero flow holds the bed**, bit for bit, however hard its chemistry
        /// would release heat: with nothing flowing nothing reacts into it. The
        /// fluid standing in it is at the bed's temperature with the products of
        /// complete conversion (`τ = +∞`), the bed is not HELD, and nothing in the
        /// tick is NaN. A fire on it then heats it by exactly `F·dt/C`.
        #[test]
        fn a_bed_with_no_flow_holds_its_temperature() {
            let slate = two_cut_slate();
            let feed = Composition::pure(2, 0);
            let (start, capacity, dt) = (Kelvin(650.0), 4.0e5, 1.0);
            let reactions = FirstOrder {
                k_ref: 0.002,
                t_ref: 650.0,
                e_over_r: 15_000.0,
                h: -3.0e5,
            };
            let (mut g, bed, fe, pe) = bed_plant(&feed, Kelvin(600.0), start, capacity, 1.0);
            let flows = flows(fe, pe, 0.0);
            let mut previous = NodeStates::default();
            for _ in 0..10 {
                previous = step(&mut g, &slate, &flows, &reactions, &previous, dt);
            }
            assert_eq!(
                bed_temperature(&g, bed).value().to_bits(),
                start.value().to_bits()
            );
            let pass = previous.bed[&bed];
            assert_eq!(pass.reaction_heat.value().to_bits(), 0.0_f64.to_bits());
            assert!(pass.residence_time.is_none());
            assert_eq!(previous.temperature[&bed], start);
            assert_eq!(previous.composition[&bed].fractions(), &[0.0, 1.0]);
            assert!(!previous.held.contains(&bed));

            let fire = 1.0e4;
            g.node_mut(bed).heat_input = Watt(fire);
            let _ = step(&mut g, &slate, &flows, &reactions, &previous, dt);
            let expected = start.value() + fire * dt / capacity;
            assert_eq!(bed_temperature(&g, bed).value(), expected);
        }

        /// **A heat-ABSORBING reaction settles below its feed** — the FCC
        /// argument of §62.0, as a gate: the bed cools as it reacts, reacts
        /// slower, and comes to rest where the feed's heat in meets the products'
        /// heat out and the heat the reaction absorbs,
        /// `ṁ·cp_A·(T_in − T_ref) = ṁ·f·X(T)·h + ṁ·cp_out(X)·(T − T_ref)`,
        /// solved here by bisection, independently of the step.
        #[test]
        fn a_heat_absorbing_bed_settles_below_its_feed() {
            let slate = two_cut_slate();
            let feed = Composition::pure(2, 0);
            let (t_in, capacity, volume, mass_flow, dt) = (650.0, 4.0e5, 1.0, 2.0, 1.0);
            let reactions = FirstOrder {
                k_ref: 0.01,
                t_ref: 650.0,
                e_over_r: 15_000.0,
                h: 2.0e5,
            };
            let (mut g, bed, fe, pe) =
                bed_plant(&feed, Kelvin(t_in), Kelvin(t_in), capacity, volume);
            let flows = flows(fe, pe, mass_flow);
            let mut previous = NodeStates::default();
            for _ in 0..5000 {
                previous = step(&mut g, &slate, &flows, &reactions, &previous, dt);
            }

            let tau = volume * FEED_DENSITY / mass_flow;
            let tref = T_REF.value();
            let balance = |t: f64| {
                let x = reactions.conversion(t, tau);
                let cp_out = (1.0 - x) * FEED_CP + x * PRODUCT_CP;
                FEED_CP * (t_in - tref) - x * reactions.h - cp_out * (t - tref)
            };
            let (mut low, mut high) = (300.0, t_in);
            assert!(balance(low) > 0.0 && balance(high) < 0.0);
            for _ in 0..200 {
                let mid = 0.5 * (low + high);
                if balance(mid) > 0.0 {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            let settled = bed_temperature(&g, bed).value();
            assert!(
                settled < t_in - 20.0,
                "a heat-absorbing bed must settle well below its {t_in} K feed, at {settled} K"
            );
            assert!(
                (settled - low).abs() < 1e-6,
                "the bed settled at {settled} K; the hand balance has it at {low} K"
            );
        }
    }

    /// M59.3 (docs/DESIGN.md §64.12): a demand a few ulps under zero — a bed
    /// fed products, reversing almost nothing by rounding — is zero, and a
    /// real negative is still refused.
    mod hydrogen_demand_band {
        use super::*;
        use crate::components::PseudoComponent;
        use crate::graph::HydrogenConsumption;

        fn two_lumps() -> Slate {
            let cut = |name: &str, tb: f64| PseudoComponent {
                name: name.into(),
                tb: Kelvin(tb),
                molar_mass: KgPerMol(0.1),
                density: Some(KgPerM3(800.0)),
                cp: JPerKgK(3200.0),
                cp_shape: None,
                phase: Phase::Liquid,
                dh_vap: None,
                cp_vapour: None,
                free_water: false,
            };
            Slate::new(vec![cut("heavy", 600.0), cut("light", 400.0)]).unwrap()
        }

        /// The demand of `products` formed from a 25/75 feed, 1 kg/s of oil.
        fn demand(products: [f64; 2]) -> Result<f64, SimError> {
            let slate = two_lumps();
            let declared = HydrogenConsumption {
                uptake: vec![0.020, 0.035],
                rate: 0.0,
            };
            let feed = Composition::from_weights(&[0.25, 0.75]).unwrap();
            let products = Composition::from_weights(&products).unwrap();
            let gas = Composition::pure(2, 0);
            pass_consumption(
                "bed",
                (&slate, &ConstantEnthalpyStub),
                &declared,
                (&feed, &products, 1.0),
                (&gas, Pascal(1e7)),
                0.0,
                Kelvin(650.0),
            )
            .map(|c| c.demand.value())
        }

        #[test]
        fn a_rounding_sized_negative_is_zero() {
            // Light turned back into heavy by one part in 1e16: the raw sum is
            // negative, by about 1.7e-18 kg/s, under the band's ~2.2e-17.
            let products = [0.25 + 1e-16, 0.75 - 1e-16];
            let raw = (products[0] - 0.25) * 0.020 + (products[1] - 0.75) * 0.035;
            assert!(raw < 0.0, "the fixture must reverse a little: {raw}");
            assert_eq!(demand(products).unwrap(), 0.0);
        }

        #[test]
        fn a_real_negative_is_refused() {
            let e = demand([0.25 + 1e-9, 0.75 - 1e-9]).unwrap_err().to_string();
            assert!(e.contains("would consume"), "{e}");
        }

        #[test]
        fn a_positive_demand_is_untouched() {
            let products = [0.25 - 1e-3, 0.75 + 1e-3];
            let by_hand = -1e-3 * 0.020 + 1e-3 * 0.035;
            assert!((demand(products).unwrap() - by_hand).abs() < 1e-18);
        }
    }
}
