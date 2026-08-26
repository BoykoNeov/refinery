//! Control algorithms — implementations of `core::traits::Controller` (M8).
//!
//! The seam's first inhabitant is a plain proportional loop. The integral term,
//! its anti-windup clamp and the MANUAL→AUTO back-calculation are M8.3, and they
//! are deliberately a separate slice: the P loop's steady-state offset is half of
//! the disturbance-rejection gate's discriminating pair (docs/DESIGN.md §10 fork
//! 6, gate 3), and shipping it alone is what makes the other half mean anything.

use refinery_core::error::SimError;
use refinery_core::graph::ControlledValue;
use refinery_core::traits::Controller;
use refinery_core::units::Seconds;

/// Proportional control: `u = clamp(K·e, 0, 1)`, with `e` the error the loop's
/// variable defines (`ControlledValue::error`).
///
/// **No bias, no manual reset, and no memory of any kind — which is the point.**
/// The textbook manual-reset form `u = u_b + K·e` would hide this controller's
/// defining property behind a tuning constant: a P loop cannot hold a setpoint
/// against a load, and the size of the offset it settles at is the signal M8.3's
/// gate reads. With a bias that offset becomes a function of how well the bias
/// was chosen rather than of the missing integral, so the discriminating half of
/// the pair would be measuring the wrong thing. `initial_output` therefore belongs
/// to M8.3, where fork 5 puts it: it is *the loop's memory*, and this controller
/// has none for it to be the initial condition of.
///
/// A consequence to expect rather than to debug: on a tank's outlet valve this
/// loop settles ABOVE its setpoint, by however much error it takes for `K·e` to
/// open the valve enough to pass the inflow. That is a proportional loop working
/// correctly.
///
/// **Direct acting, and the sign lives in `ControlledValue::error`, not here.**
/// Positive error means above setpoint, and a positive gain opens the actuator —
/// correct for a valve that DRAINS the measured tank. A loop whose actuator fills
/// the tank would need reverse action, which is not expressible today and is not
/// silently available either: a negative gain is refused at load, because the one
/// wiring this slice builds is the draining one and a reverse-acting loop needs
/// its own declaration rather than a sign.
#[derive(Debug, Clone, Copy)]
pub struct ProportionalController {
    /// Proportional gain, in reciprocal units of the measured variable — `1/m`
    /// for a level loop, which is why the scenario key says `gain_per_m`.
    gain: f64,
}

impl ProportionalController {
    /// `gain` must be finite and > 0; see the type's note on reverse action.
    ///
    /// # Errors
    /// `SimError::Scenario` on a non-finite or non-positive gain. There is no
    /// default gain, for the reason `x_T` has none (§3a fork 6): a silent default
    /// is an invented value in disguise, and every gate would then pass for
    /// whatever was chosen.
    pub fn new(gain: f64) -> Result<Self, SimError> {
        if !gain.is_finite() || gain <= 0.0 {
            return Err(SimError::Scenario(format!(
                "proportional gain must be finite and > 0, got {gain}. A zero gain is a \
                 loop that does nothing, and a negative one is reverse action, which this \
                 fidelity does not express — see `ProportionalController`"
            )));
        }
        Ok(Self { gain })
    }
}

impl Controller for ProportionalController {
    fn name(&self) -> &'static str {
        "proportional"
    }

    fn update(
        &mut self,
        measurement: ControlledValue,
        setpoint: ControlledValue,
        _dt: Seconds,
    ) -> Result<f64, SimError> {
        // The error term comes from `core`, which owns the sign convention. An
        // impl differencing the two itself would be free to disagree with the
        // value a snapshot reader reconstructs from the same two reported numbers.
        let error = ControlledValue::error(measurement, setpoint);
        if !error.is_finite() {
            return Err(SimError::NonFiniteState {
                location: format!(
                    "proportional controller error term (measurement {}, setpoint {})",
                    measurement.magnitude(),
                    setpoint.magnitude()
                ),
            });
        }
        // Clamped here rather than by the caller, because saturation is what
        // M8.3's anti-windup has to know about and that knowledge belongs with
        // the algorithm. The engine re-checks the range and refuses rather than
        // trusting this.
        Ok((self.gain * error).clamp(0.0, 1.0))
    }
}
