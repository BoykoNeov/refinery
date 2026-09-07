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
    /// **It is not a phase marker and it is not `docs/DEFERRED.md` B3.** B3 is
    /// a stream that is PART liquid and PART vapour, which needs a quality on
    /// `Composition` and changes every reader of it. This is a single-phase
    /// vapour stream declaring one scalar about its own energy — and spelling
    /// it `phase: Phase` instead would contradict `Composition::phase`, since
    /// a boil-off vent's cuts are ones the slate *declares* `Liquid`
    /// (docs/DESIGN.md §15 fork 1).
    ///
    /// Written today by exactly one producer — a boil-off vent, from the
    /// `BoilOff::latent_heat` the model sized the flash against. It is signed
    /// by the mass flow rather than by itself, so a condensing stream (B12)
    /// arrives carrying `Some(λ)` and gives it up, with no rename.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub latent: Option<JPerKg>,
}

impl Stream {
    pub fn stagnant(slate_len: usize, temperature: Kelvin, pressure: Pascal) -> Self {
        Self {
            mass_flow: KgPerSec::ZERO,
            temperature,
            pressure,
            composition: Composition::pure(slate_len, 0),
            latent: None,
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
    }
}
