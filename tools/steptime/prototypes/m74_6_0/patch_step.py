import sys

p = sys.argv[1]
s = open(p, encoding="utf-8").read()


def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (s.count(old), old[:100])
    s = s.replace(old, new)


# --- constants
rep("""/// Newton steps before a tick's step is refused.""",
"""/// The liquid a tray holding none is read its hydraulic slope over [kg]
/// (M74.6.0 probe): an empty tray's relative step is zero.
const EMPTY_SLOPE_KG: f64 = 1.0;

/// Newton steps before a tick's step is refused.""")

# --- SetFlows fields
rep("""    bottoms_before_limits: f64,
    reboiler_before_limits: f64,
}

/// One evaluation of the residuals.""",
"""    bottoms_before_limits: f64,
    reboiler_before_limits: f64,
    /// M74.6.0 probe: each stage's start-of-tick mole fractions, the drum's
    /// (an empty stage's the last liquid it held, a label with no moles),
    /// each stage's steel's heat capacity [J/K] and its start-of-tick
    /// temperature [K].
    labels: Vec<Vec<f64>>,
    drum_label: Vec<f64>,
    steel: Vec<f64>,
    previous_temperature: Vec<f64>,
}

/// One evaluation of the residuals.""")

rep("""    liquid: Vec<Vec<f64>>,
    k: Vec<Vec<f64>>,
    flows: Flows,
    drum_temperature: Kelvin,
    condenser_duty: f64,
}""", """    liquid: Vec<Vec<f64>>,
    drum_liquid: Vec<f64>,
    k: Vec<Vec<f64>>,
    flows: Flows,
    drum_temperature: Kelvin,
    condenser_duty: f64,
}""")

# --- steel helper
rep("""    /// The tick: the liquid flows set, then Newton on the temperatures and
    /// vapours from `start`.""",
"""    /// M74.6.0 probe: each stage's steel's heat capacity [J/K]: a tray's
    /// shell over its spacing and its deck, the base's shell around its
    /// volume and a head (M74.0's game estimate); zero with the steel off.
    fn steel_capacity(&self) -> Result<Vec<f64>, SimError> {
        let n = self.plan.stages;
        if !crate::probe::steel() || self.spec.trays.is_empty() {
            return Ok(vec![0.0; n]);
        }
        use crate::probe::{DECK_KG_M2, SHELL_M, STEEL_CP, STEEL_DENSITY};
        let pi = std::f64::consts::PI;
        let mut steel = Vec::with_capacity(n);
        let mut diameter = 0.0;
        for j in 0..n - 1 {
            let section = crate::trays::section_for(self.spec, j as u32 + 1)?;
            diameter = section.diameter.value();
            let area = pi * diameter * diameter / 4.0;
            let shell = pi * diameter * section.spacing.value() * SHELL_M * STEEL_DENSITY;
            steel.push((shell + area * DECK_KG_M2) * STEEL_CP);
        }
        // The base: the last tray's diameter, as tall as its volume asks.
        let area = pi * diameter * diameter / 4.0;
        let height = self.holdup.base_volume.value() / area;
        let shell = (pi * diameter * height + area) * SHELL_M * STEEL_DENSITY;
        steel.push(shell * STEEL_CP);
        Ok(steel)
    }

    /// The tick: the liquid flows set, then Newton on the temperatures and
    /// vapours from `start`.""")

# --- feasible restart
rep("""        let mut evaluation = self.evaluate(&u, &amounts, &drum, &set)?;""",
"""        // M74.6.0 probe: a hint the state cannot reach (a stage it would
        // empty past nothing): Newton from the state's temperatures and no
        // vapour instead; only where the tick would be refused.
        let mut evaluation = match self.evaluate(&u, &amounts, &drum, &set) {
            Ok(evaluation) => evaluation,
            Err(refused) => {
                let mut restart: Vec<f64> =
                    state.stage_temperature.iter().map(|t| t.value()).collect();
                restart.extend(std::iter::repeat(0.0).take(n));
                match self.evaluate(&restart, &amounts, &drum, &set) {
                    Ok(evaluation) => {
                        u = restart;
                        evaluation
                    }
                    Err(_) => return Err(refused),
                }
            }
        };""")

# --- labels in set_flows
rep("""        let stage_x: Vec<Vec<f64>> = amounts
            .iter()
            .map(|a| fractions(a))
            .collect::<Result<_, _>>()?;
        let drum_x = fractions(drum)?;""",
"""        // M74.6.0 probe: a stage or drum holding nothing reads the last
        // liquid it held: a label, no moles.
        let labelled = |a: &[f64], held: &Composition| -> Result<Vec<f64>, SimError> {
            if a.iter().sum::<f64>() == 0.0 {
                Ok(MoleFractions::from_mass(held, self.slate)?.fractions().to_vec())
            } else {
                fractions(a)
            }
        };
        let stage_x: Vec<Vec<f64>> = amounts
            .iter()
            .zip(&state.stage_composition)
            .map(|(a, held)| labelled(a, held))
            .collect::<Result<_, _>>()?;
        let drum_x = labelled(drum, &state.drum_composition)?;
        let total_reflux = crate::probe::total_reflux();""")

rep("""                if i != 0 && i != last {
                    side[*j] = plan.mass_ratios[i] * self.feed_mass;""",
"""                if i != 0 && i != last && !total_reflux {
                    side[*j] = plan.mass_ratios[i] * self.feed_mass;""")

rep("""            let more = Kg(state.stage_liquid[j].value() * (1.0 + SLOPE_DIFFERENCE));""",
"""            let more = if state.stage_liquid[j].value() > 0.0 {
                Kg(state.stage_liquid[j].value() * (1.0 + SLOPE_DIFFERENCE))
            } else {
                Kg(EMPTY_SLOPE_KG)
            };""")

rep("""        if base_level < LOW_LEVEL {
            reboiler *= (base_level / LOW_LEVEL).max(0.0);
            base_limit = LevelLimit::Dry;
        }""",
"""        if base_level < LOW_LEVEL {
            reboiler *= (base_level / LOW_LEVEL).max(0.0);
            // M74.6.0 probe: the bottoms pump loses its suction too.
            bottoms_mass *= (base_level / LOW_LEVEL).max(0.0);
            base_limit = LevelLimit::Dry;
        }""")

rep("""        let distillate = (controlled * drum_out_share + overflow) / mean(&drum_x);
        let reflux = self.spec.reflux_ratio * controlled * drum_out_share / mean(&drum_x);""",
"""        // M74.6.0 probe, total reflux: no distillate but a full drum's
        // overflow, and all the drum's loop asks sent back as reflux.
        let (distillate, reflux) = if total_reflux {
            (
                overflow / mean(&drum_x),
                (self.spec.reflux_ratio + 1.0) * controlled * drum_out_share / mean(&drum_x),
            )
        } else {
            (
                (controlled * drum_out_share + overflow) / mean(&drum_x),
                self.spec.reflux_ratio * controlled * drum_out_share / mean(&drum_x),
            )
        };""")

rep("""            base_integral,
            bottoms_before_limits,
            reboiler_before_limits,
        })
    }""",
"""            base_integral,
            bottoms_before_limits,
            reboiler_before_limits,
            labels: stage_x,
            drum_label: drum_x,
            steel: self.steel_capacity()?,
            previous_temperature: state.stage_temperature.iter().map(|t| t.value()).collect(),
        })
    }""")

# --- evaluate: the liquid down capped
rep("""                down[j] = (off - set.drawn[j] - out[j]).max(0.0);
                liquid_in[j + 1] = down[j];
            }
            held_next.push(held[j] + dt * (arriving - down[j] - set.drawn[j] - out[j] - vapour[j]));
        }
        let drum_held: f64 = drum.iter().sum();
        let drum_next = drum_held + dt * (vapour[0] - set.reflux - set.distillate);
        if let Some(j) = held_next.iter().position(|h| h.is_nan() || *h <= 0.0) {""",
"""                down[j] = (off - set.drawn[j] - out[j]).max(0.0);
            }
            let mut next = held[j] + dt * (arriving - down[j] - set.drawn[j] - out[j] - vapour[j]);
            // M74.6.0 probe: a tray the liquid going down would empty past
            // nothing within the tick passes only what it has: it ends the
            // tick empty, a state the step holds.
            if next < 0.0 && down[j] > 0.0 {
                let short = -next / dt;
                if short <= down[j] {
                    down[j] -= short;
                    next = 0.0;
                } else {
                    down[j] = 0.0;
                    next = held[j] + dt * (arriving - set.drawn[j] - out[j] - vapour[j]);
                }
            }
            if j + 1 < n {
                liquid_in[j + 1] = down[j];
            }
            held_next.push(next);
        }
        let drum_held: f64 = drum.iter().sum();
        let drum_next = drum_held + dt * (vapour[0] - set.reflux - set.distillate);
        if let Some(j) = held_next.iter().position(|h| h.is_nan() || *h < 0.0) {""")
rep("""        if drum_next.is_nan() || drum_next <= 0.0 {""",
"""        if drum_next.is_nan() || drum_next < 0.0 {""")

# --- Thomas identity rows
rep("""            let solved = thomas(&lower, &diagonal, &upper, &rhs)""",
"""            // M74.6.0 probe: a row holding nothing with nothing coming or
            // going (an empty stage, an empty drum) holds no moles.
            for r in 0..rows {
                if diagonal[r] == 0.0 && lower[r] == 0.0 && upper[r] == 0.0 && rhs[r] == 0.0 {
                    diagonal[r] = 1.0;
                }
            }
            let solved = thomas(&lower, &diagonal, &upper, &rhs)""")

# --- normalise with labels
rep("""        let liquid: Vec<Vec<f64>> = x.iter().map(|r| normalise(r)).collect::<Result<_, _>>()?;
        let drum_liquid = normalise(&x_drum)?;""",
"""        let empty = |row: &[f64]| row.iter().sum::<f64>() == 0.0;
        let liquid: Vec<Vec<f64>> = x
            .iter()
            .zip(&set.labels)
            .map(|(r, label)| if empty(r) { Ok(label.clone()) } else { normalise(r) })
            .collect::<Result<_, _>>()?;
        let drum_liquid = if empty(&x_drum) {
            set.drum_label.clone()
        } else {
            normalise(&x_drum)?
        };""")
rep("""        let drum_temperature = bubble_temperature_unnormalized(
            slate,
            self.thermo,
            &drum_liquid,
            self.pressure,
            "the reflux drum",
        )?;""",
"""        let drum_temperature = if empty(&x_drum) {
            self.state.drum_temperature
        } else {
            bubble_temperature_unnormalized(
                slate,
                self.thermo,
                &drum_liquid,
                self.pressure,
                "the reflux drum",
            )?
        };""")
rep("""            let stored = (held_next[j] * h[j] - held[j] * set.enthalpy[j]) / dt;""",
"""            let mut stored = (held_next[j] * h[j] - held[j] * set.enthalpy[j]) / dt;
            // M74.6.0 probe: the stage's steel, at the stage's temperature.
            if set.steel[j] > 0.0 {
                stored += set.steel[j] * (temperatures[j] - set.previous_temperature[j]) / dt;
            }""")
rep("""            liquid,
            k,
            flows,
            drum_temperature,
            condenser_duty,
        })
    }""", """            liquid,
            drum_liquid,
            k,
            flows,
            drum_temperature,
            condenser_duty,
        })
    }""")

# --- finish
rep("""        let held_mass = |moles: &[f64]| -> Result<(Kg, Composition), SimError> {
            let masses = to_mass(moles);
            Ok((Kg(masses.iter().sum()), Composition::from_weights(&masses)?))
        };
        let mut stage_liquid = Vec::with_capacity(n);
        let mut stage_composition = Vec::with_capacity(n);
        for a in &evaluation.amounts {
            let (mass, composition) = held_mass(a)?;
            stage_liquid.push(mass);
            stage_composition.push(composition);
        }
        let (drum_liquid, drum_composition) = held_mass(&evaluation.drum_amounts)?;""",
"""        // M74.6.0 probe: an empty stage or drum keeps its label.
        let held_mass = |moles: &[f64], label: &Composition| -> Result<(Kg, Composition), SimError> {
            let masses = to_mass(moles);
            let total: f64 = masses.iter().sum();
            if total == 0.0 {
                return Ok((Kg(0.0), label.clone()));
            }
            Ok((Kg(total), Composition::from_weights(&masses)?))
        };
        let mut stage_liquid = Vec::with_capacity(n);
        let mut stage_composition = Vec::with_capacity(n);
        for (a, label) in evaluation.amounts.iter().zip(&self.state.stage_composition) {
            let (mass, composition) = held_mass(a, label)?;
            stage_liquid.push(mass);
            stage_composition.push(composition);
        }
        let (drum_liquid, drum_composition) =
            held_mass(&evaluation.drum_amounts, &self.state.drum_composition)?;""")
rep("""            drum_liquid: MoleFractions::from_amounts(&evaluation.x_drum)?
                .fractions()
                .to_vec(),""", """            drum_liquid: evaluation.drum_liquid.clone(),""")
open(p, "w", encoding="utf-8").write(s)
print("ok")
