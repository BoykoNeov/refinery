//! Control algorithms — implementations of `core::traits::Controller` (M8).
//!
//! Two inhabitants, and the order they landed in is load-bearing. M8.2 shipped a
//! plain proportional loop ALONE, because its steady-state offset is half of the
//! disturbance-rejection gate's discriminating pair (docs/DESIGN.md §10 fork 6,
//! gate 3) and a pair needs both halves measured on the same plant. M8.3 adds the
//! integral term, its anti-windup clamp and the MANUAL→AUTO back-calculation —
//! the latter two being **the same arithmetic**, written once as
//! `PiController::back_calculate` and reached from both directions.

use refinery_core::error::SimError;
use refinery_core::graph::{ControlAction, ControlledValue};
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
/// **Either direction, and the sign lives in `ControlledValue::error`, not
/// here.** A positive error raises the output, and the LOOP's declared
/// `ControlAction` says what a positive error is. The rule, in its fourth wording
/// (docs/DESIGN.md §22): **a DIRECT loop's output must lower its measurement as it
/// rises — a drain, a vent, a cooler — and a REVERSE loop's must raise it — a
/// furnace.** (M8.4 wrote "a level loop must actuate a drain", M10 "an outlet of
/// the measured holdup", §21 "raising the output must lower the measurement";
/// each was a special case of the direct half.) This impl sees the action only as
/// an argument it hands straight to `error`, and a negative gain stays refused:
/// it would be a second way to say "reverse", and one the loader cannot check
/// against the actuator.
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
                 loop that does nothing, and a negative one is reverse action written as a \
                 sign. Reverse action is DECLARED on the loop (`action = \"reverse\"`), and \
                 a second way to say it would be one the loader cannot check — see \
                 `ControlAction`"
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
        action: ControlAction,
        _dt: Seconds,
    ) -> Result<f64, SimError> {
        // The error term comes from `core`, which owns the sign convention. An
        // impl differencing the two itself would be free to disagree with the
        // value a snapshot reader reconstructs from the same two reported numbers.
        let error = ControlledValue::error(measurement, setpoint, action);
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

    /// A no-op, written out rather than inherited.
    ///
    /// `u = K·e` is a pure function of the current error: there is no memory to
    /// seed, so the next `update` returns what the error says regardless of what
    /// the actuator was left at, and MANUAL→AUTO on a P loop steps the valve by
    /// however much the human's position differed from `K·e`. That is the
    /// controller being what it is rather than a transfer defect — and it is
    /// precisely why fork 5 puts `initial_output` on the PI loop and refuses it
    /// here.
    ///
    /// The trait deliberately has no default body, so this arm is a decision an
    /// impl makes rather than one it inherits by forgetting.
    fn seed_from_output(
        &mut self,
        _output: f64,
        _measurement: ControlledValue,
        _setpoint: ControlledValue,
        _action: ControlAction,
    ) -> Result<(), SimError> {
        Ok(())
    }
}

/// Proportional + integral control: `u = clamp(K·e + b, 0, 1)`, where `b` is the
/// loop's memory and `e` is the error `ControlledValue::error` defines.
///
/// **This is the first object in the project that carries state across ticks
/// behind a trait**, which is the property §3a fork 5 named as the thing that
/// turns an element into a controller. Two consequences follow from that alone:
/// rule 3 makes `b` part of the answer rather than part of the path (the M8.0
/// lesson — a carried-over number decides where the next pass terminates), and
/// fork 5 therefore makes it an **initial condition the scenario declares**.
///
/// **The memory is held in output units, not in error-seconds, and that choice is
/// what makes anti-windup and bumpless transfer one piece of arithmetic.** The
/// textbook form carries `∫e dt` and multiplies by `K/T_i` at use; this one folds
/// the gains in at accumulation time, so `b` *is* the share of the actuator
/// position the integral term is responsible for. Inverting the algorithm for
/// "what memory would make the next output be `u`" is then `b = u − K·e` — one
/// line, [`PiController::back_calculate`], reached from all three places a loop's
/// memory is ever written:
///
/// - the anti-windup clamp, when the actuator cannot answer;
/// - MANUAL→AUTO transfer, from the position a human left the valve at;
/// - load, from the declared `initial_output`.
///
/// The alternative — an `∫e dt` state, a clamp on it, and a separate transfer
/// formula — is the same behaviour written three times, and the roadmap's "built
/// once rather than twice" would have been false.
///
/// **The integral is evaluated on errors already accumulated (explicit Euler),
/// not on this tick's.** `u` uses the `b` standing at the top of the tick and the
/// error is added afterwards, which is the engine's own integration rule and has
/// one property a gate reads directly: the first `update` after a seed returns
/// the seeded output, so a MANUAL→AUTO transfer moves the valve by nothing rather
/// than by one integration step.
///
/// **Anti-windup is conditional integration performed BY the back-calculation.**
/// While the output is on a limit, `b` is not accumulated; it is set to whatever
/// makes the clamped position the answer. So the loop tracks its own saturated
/// actuator, and the instant the error falls far enough the output leaves the
/// limit smoothly, with no accumulated surplus to spend first. Without it, `b`
/// grows for as long as the plant cannot answer, and the signature is an
/// undershoot after the load is removed — which is what gate 4 measures, on a
/// plant built to saturate.
///
/// Either direction, like [`ProportionalController`] and by the same route: the
/// sign lives in `ControlledValue::error`, and the loop's declared
/// `ControlAction` is passed to it by all three writers of the memory — the
/// load-time seed in `new`, the MANUAL→AUTO seed, and `update` (whose clamp
/// branch back-calculates with the same signed error it just used). A seed taken
/// against the other sign would step the first output by `2·K·e`.
///
/// **Not `Clone` and not `Copy`, unlike [`ProportionalController`]**, and the
/// asymmetry is M8.2's correction 5 one level down. That correction took `Clone`
/// off `PlantGraph` because copying a graph would fork a loop's memory into two
/// engines that then diverge — and it did so with no call site in the workspace,
/// on the strength of the hazard alone. A `Copy` derive here reopens exactly that
/// hazard: `let mut c = *controller` would fork the integral silently, and the
/// copy would go on integrating a plant it is no longer attached to. The trait
/// object blocks that path today, which is the same "unreachable" the earlier
/// correction declined to rely on. A stateless controller has nothing to fork and
/// keeps its derives.
#[derive(Debug)]
pub struct PiController {
    /// Proportional gain `K`, in reciprocal units of the measured variable —
    /// `1/m` for a level loop, which is why the scenario key says `gain_per_m`.
    gain: f64,
    /// Integral time `T_i` [s]: the time in which the integral term alone repeats
    /// the proportional term's contribution to the output. The ISA "reset time",
    /// which is why the scenario key carries seconds.
    integral_time_s: f64,
    /// The loop's memory: the share of the actuator position the integral term is
    /// responsible for, dimensionless like the position itself.
    ///
    /// Never written except through [`PiController::back_calculate`] or the
    /// accumulation in `update`, and never *born* zero — `new` requires the
    /// declared `initial_output` and derives this from it, so fork 5's "one
    /// declared number, no silent zero" holds by construction rather than by the
    /// loader remembering to call something afterwards.
    integral: f64,
}

impl PiController {
    /// Build a loop's algorithm and seed its memory from the declared
    /// `initial_output`.
    ///
    /// The measurement and setpoint standing at load are required arguments
    /// rather than a call the caller makes next, so an unseeded `PiController` is
    /// not a value that can exist. `initial_output` is an actuator position and is
    /// validated as one, by the rule `Command::SetValveOpening` already applies to
    /// the same quantity.
    ///
    /// # Errors
    /// `SimError::Scenario` on a non-finite or non-positive gain or integral time
    /// — a zero integral time is a division by zero wearing a tuning constant's
    /// clothes — or on an `initial_output` that is not a finite fraction in
    /// `[0, 1]`. No defaults, for the reason `x_T` has none (§3a fork 6): a silent
    /// default is an invented value in disguise, and every gate would then pass
    /// for whatever was chosen.
    pub fn new(
        gain: f64,
        integral_time_s: f64,
        initial_output: f64,
        measurement: ControlledValue,
        setpoint: ControlledValue,
        action: ControlAction,
    ) -> Result<Self, SimError> {
        if !gain.is_finite() || gain <= 0.0 {
            return Err(SimError::Scenario(format!(
                "proportional gain must be finite and > 0, got {gain}. A zero gain is a \
                 loop that does nothing, and a negative one is reverse action written as a \
                 sign. Reverse action is DECLARED on the loop (`action = \"reverse\"`), and \
                 a second way to say it would be one the loader cannot check — see \
                 `ControlAction`"
            )));
        }
        if !integral_time_s.is_finite() || integral_time_s <= 0.0 {
            return Err(SimError::Scenario(format!(
                "integral time must be finite and > 0 s, got {integral_time_s}. It divides \
                 the gain, so zero is not 'no integral action' — that is `algorithm = \"p\"` \
                 — and a negative one integrates the error away from setpoint"
            )));
        }
        let mut controller = Self {
            gain,
            integral_time_s,
            integral: 0.0,
        };
        controller.seed_from_output(initial_output, measurement, setpoint, action)?;
        Ok(controller)
    }

    /// Set the memory so that the next `update` returns `output`.
    ///
    /// **The one place `b` is solved for**, and the reason anti-windup and
    /// bumpless transfer are one piece of code: inverting `u = K·e + b` for `b` is
    /// the whole of both. It takes the error rather than the measurement/setpoint
    /// pair because the caller inside `update` already holds it, and re-deriving
    /// it here would put a second expression of the same difference in the same
    /// function.
    fn back_calculate(&mut self, output: f64, error: f64) {
        self.integral = output - self.gain * error;
    }
}

impl Controller for PiController {
    fn name(&self) -> &'static str {
        "proportional_integral"
    }

    fn update(
        &mut self,
        measurement: ControlledValue,
        setpoint: ControlledValue,
        action: ControlAction,
        dt: Seconds,
    ) -> Result<f64, SimError> {
        // The error term comes from `core`, which owns the sign convention — see
        // `ProportionalController::update` for why no impl computes its own.
        let error = ControlledValue::error(measurement, setpoint, action);
        if !error.is_finite() {
            return Err(SimError::NonFiniteState {
                location: format!(
                    "PI controller error term (measurement {}, setpoint {})",
                    measurement.magnitude(),
                    setpoint.magnitude()
                ),
            });
        }
        // The memory standing at the TOP of the tick, not one that already
        // includes this tick's error: explicit Euler, the engine's own rule, and
        // what makes the first update after a seed return the seeded position.
        let unclamped = self.gain * error + self.integral;
        if !unclamped.is_finite() {
            return Err(SimError::NonFiniteState {
                location: format!(
                    "PI controller output (gain {}, integral {})",
                    self.gain, self.integral
                ),
            });
        }
        let output = unclamped.clamp(0.0, 1.0);
        if (0.0..=1.0).contains(&unclamped) {
            // Not on a limit: accumulate this tick's error for the next one.
            // `K/T_i` folds both gains in here, which is what keeps the memory in
            // output units — see the type's note.
            self.integral += self.gain / self.integral_time_s * error * dt.value();
            if !self.integral.is_finite() {
                return Err(SimError::NonFiniteState {
                    location: format!(
                        "PI controller integral term (dt {} s, error {error})",
                        dt.value()
                    ),
                });
            }
        } else {
            // On a limit: the actuator cannot answer, so accumulating against it
            // is exactly the windup this branch refuses. The memory becomes
            // whatever makes the position actually written the answer — the same
            // inversion MANUAL→AUTO performs, which is why the two are one piece
            // of arithmetic rather than two that can disagree.
            self.back_calculate(output, error);
        }
        Ok(output)
    }

    fn seed_from_output(
        &mut self,
        output: f64,
        measurement: ControlledValue,
        setpoint: ControlledValue,
        action: ControlAction,
    ) -> Result<(), SimError> {
        if !output.is_finite() || !(0.0..=1.0).contains(&output) {
            return Err(SimError::Scenario(format!(
                "a PI loop's memory is seeded from an actuator position, and {output} is \
                 not a finite fraction in [0, 1] — the range `Command::SetValveOpening` \
                 already enforces on the same quantity"
            )));
        }
        let error = ControlledValue::error(measurement, setpoint, action);
        if !error.is_finite() {
            return Err(SimError::NonFiniteState {
                location: format!(
                    "PI controller seed error term (measurement {}, setpoint {})",
                    measurement.magnitude(),
                    setpoint.magnitude()
                ),
            });
        }
        self.back_calculate(output, error);
        Ok(())
    }
}
