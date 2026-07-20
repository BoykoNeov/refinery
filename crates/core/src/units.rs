//! SI unit newtypes. A bare `f64` must never cross a public API boundary.
//!
//! Deliberately minimal: transparent wrappers with arithmetic where it is
//! dimensionally honest, `value()` escape hatch for math kernels. If this
//! becomes limiting, evaluate the `uom` crate — but only with a benchmark
//! and an ergonomics review, not by default.

use serde::{Deserialize, Serialize};

macro_rules! unit {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(pub f64);

        impl $name {
            pub const ZERO: Self = Self(0.0);
            #[inline]
            pub fn value(self) -> f64 { self.0 }
            #[inline]
            pub fn is_finite(self) -> bool { self.0.is_finite() }
        }

        impl core::ops::Add for $name {
            type Output = Self;
            #[inline] fn add(self, rhs: Self) -> Self { Self(self.0 + rhs.0) }
        }
        impl core::ops::Sub for $name {
            type Output = Self;
            #[inline] fn sub(self, rhs: Self) -> Self { Self(self.0 - rhs.0) }
        }
        impl core::ops::Mul<f64> for $name {
            type Output = Self;
            #[inline] fn mul(self, rhs: f64) -> Self { Self(self.0 * rhs) }
        }
        impl core::ops::Div<f64> for $name {
            type Output = Self;
            #[inline] fn div(self, rhs: f64) -> Self { Self(self.0 / rhs) }
        }
    };
}

unit!(/// Absolute pressure [Pa]. Kelvin
    Pascal);
unit!(/// Temperature [K], absolute.
    Kelvin);
unit!(/// Mass flow [kg/s].
    KgPerSec);
unit!(/// Mass [kg].
    Kg);
unit!(/// Volume [m³].
    CubicMeter);
unit!(/// Volumetric flow [m³/s].
    M3PerSec);
unit!(/// Density [kg/m³].
    KgPerM3);
unit!(/// Length [m].
    Meter);
unit!(/// Area [m²].
    SquareMeter);
unit!(/// Time [s].
    Seconds);
unit!(/// Energy [J].
    Joule);
unit!(/// Power [W].
    Watt);
unit!(/// Specific heat capacity [J/(kg·K)].
    JPerKgK);
unit!(/// Specific enthalpy [J/kg]. Used for a reaction's heat `Δh_rxn`.
    ///
    /// SIGN CONVENTION: positive = ENDOTHERMIC (heat ABSORBED by the reaction).
    /// A reactor holding its setpoint must then SUPPLY `ṁ·Δh_rxn` on top of the
    /// sensible change, which is why the reported physical duty is
    /// `sensible + ṁ·Δh_rxn` (docs/DESIGN.md §5, `traits::ReactionModel`). FCC
    /// cracking is endothermic, so its lumps carry a positive `Δh_rxn`.
    JPerKg);
unit!(/// Overall heat transfer coefficient times area, `UA` [W/K].
    ///
    /// Deliberately ONE number rather than a separate `U` and `A`: at this
    /// fidelity nothing distinguishes them — no geometry, no wind, no insulation
    /// model, no radiation — so splitting them would invent two quantities the
    /// model cannot tell apart (docs/DESIGN.md §4a).
    WattPerKelvin);
unit!(/// Molar mass [kg/mol].
    KgPerMol);

/// Standard gravity [m/s²].
pub const G: f64 = 9.806_65;
/// Standard atmospheric pressure.
pub const P_ATM: Pascal = Pascal(101_325.0);
/// Ambient default temperature.
pub const T_AMBIENT: Kelvin = Kelvin(293.15);

// Dimensionally honest cross-type ops used constantly by the engine:
impl core::ops::Mul<Seconds> for KgPerSec {
    type Output = Kg;
    #[inline]
    fn mul(self, dt: Seconds) -> Kg {
        Kg(self.0 * dt.0)
    }
}
impl core::ops::Div<KgPerM3> for KgPerSec {
    type Output = M3PerSec;
    #[inline]
    fn div(self, rho: KgPerM3) -> M3PerSec {
        M3PerSec(self.0 / rho.0)
    }
}
impl core::ops::Div<KgPerM3> for Kg {
    type Output = CubicMeter;
    #[inline]
    fn div(self, rho: KgPerM3) -> CubicMeter {
        CubicMeter(self.0 / rho.0)
    }
}
