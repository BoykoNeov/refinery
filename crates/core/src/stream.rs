//! `Stream`: the state carried by every pipe (edge) in the plant graph.

use crate::components::Composition;
use crate::units::{JPerKg, Kelvin, KgPerSec, Pascal};
use serde::{Deserialize, Serialize};

/// Material stream in a pipe. Sign convention: positive `mass_flow` means
/// flow in the edge's graph direction (source node → target node); the
/// hydraulic solver may produce negative flows (reverse flow) and all
/// transport code must handle both signs.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Stream {
    pub mass_flow: KgPerSec,
    pub temperature: Kelvin,
    /// Representative pressure of the stream (edge midpoint by convention;
    /// node pressures are the authoritative hydraulic state).
    pub pressure: Pascal,
    pub composition: Composition,
    /// Latent heat carried per kilogram [J/kg] — `None` on a liquid stream.
    ///
    /// **The engine's enthalpy datum is saturated LIQUID at `T_REF`** (see
    /// `energy::T_REF`), so `h = cp·(T − T_REF)` describes a liquid and only a
    /// liquid. A vapour on that datum is a liquid plus its heat of
    /// vaporisation, and this field is that second term. A stream's specific
    /// enthalpy is therefore `cp·(T − T_REF) + latent`, which
    /// `energy::stream_enthalpy_flux` is the single owner of.
    ///
    /// **`None` rather than a bare `f64` of zero**, and the distinction is
    /// `NodeSnapshot::column_duty`'s: `None` says "this stream is a liquid and
    /// the question does not arise", where `Some(ZERO)` would say "a vapour
    /// whose latent heat is nothing". Only the first is true of every stream
    /// this engine carried before M13, and `skip_serializing_if` is then what
    /// keeps sixteen shipped plants byte-identical.
    ///
    /// **It is not a phase marker.** Spelling it `phase: Phase` would
    /// contradict `Composition::phase`, since a boil-off vent's cuts are ones
    /// the slate *declares* `Liquid` (docs/DESIGN.md §15 fork 1). Since M53 a
    /// stream that is PART vapour carries it too, and then it is the latent heat
    /// of its vapour share per kilogram of the WHOLE stream, `q·λ` — the energy
    /// term, while `vapour_fraction` below is the phase term.
    ///
    /// Two producers. A boil-off vent writes `Some(λ)`, from the
    /// `BoilOff::latent_heat` the model sized the flash against, and no
    /// `vapour_fraction`: it is all vapour by construction, and leaving that
    /// field off keeps every boil-off plant's wire unchanged. A line flash (M53,
    /// docs/DESIGN.md §58) writes `Some(q·λ)` and `vapour_fraction = Some(q)`
    /// from ONE `VapourShare`, so the two cannot describe different vapour. It is
    /// signed by the mass flow rather than by itself, so a condensing stream (B12)
    /// arrives carrying `Some(λ)` and gives it up, with no rename.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub latent: Option<JPerKg>,
    /// The share of this stream's MASS that is vapour [-], in `(0, 1]` — `None`
    /// on a stream carried as one phase (M53, docs/DESIGN.md §58).
    ///
    /// Written only by a line flash (`[fidelity] line_flash = "equilibrium"`),
    /// together with `latent`, from the upwind node's `VapourShare`. `None`
    /// rather than `Some(0.0)` for the reason `latent` gives: absent is "the
    /// question does not arise", which is every stream on a plant that does not
    /// select the model, and `skip_serializing_if` is what keeps those plants'
    /// wire byte-identical.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vapour_fraction: Option<f64>,
}

impl Stream {
    pub fn stagnant(slate_len: usize, temperature: Kelvin, pressure: Pascal) -> Self {
        Self {
            mass_flow: KgPerSec::ZERO,
            temperature,
            pressure,
            composition: Composition::pure(slate_len, 0),
            latent: None,
            vapour_fraction: None,
        }
    }

    pub fn all_finite(&self) -> bool {
        self.mass_flow.is_finite()
            && self.temperature.is_finite()
            && self.pressure.is_finite()
            && self.composition.fractions().iter().all(|f| f.is_finite())
            // An `Option` is exactly the field a finiteness sweep skips, and
            // rule 5 does not have an exemption for one: a `None` is finite by
            // construction, a `Some(NaN)` is not.
            && self.latent.is_none_or(|l| l.is_finite())
            && self.vapour_fraction.is_none_or(f64::is_finite)
    }
}
