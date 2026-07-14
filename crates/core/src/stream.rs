//! `Stream`: the state carried by every pipe (edge) in the plant graph.

use crate::components::Composition;
use crate::units::{Kelvin, KgPerSec, Pascal};
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
}

impl Stream {
    pub fn stagnant(slate_len: usize, temperature: Kelvin, pressure: Pascal) -> Self {
        Self {
            mass_flow: KgPerSec::ZERO,
            temperature,
            pressure,
            composition: Composition::pure(slate_len, 0),
        }
    }

    pub fn all_finite(&self) -> bool {
        self.mass_flow.is_finite()
            && self.temperature.is_finite()
            && self.pressure.is_finite()
            && self.composition.fractions().iter().all(|f| f.is_finite())
    }
}
