//! `SimpleLookup`: the simple-fidelity reactor — a fixed conversion table, no
//! kinetics. Selected by `reactions = "lookup"`; the M4.2 `FourLump` swaps in
//! behind the same `ReactionModel` trait with no change above it.
//!
//! The model is a table indexed by (temperature band, feed lump). A reactor
//! holds one temperature (`t_set`), so exactly one band is active per call; each
//! feed lump's mass is then redistributed across the slate by that band's row
//! for it. Two properties are load-bearing and enforced at construction:
//!
//! - **Every row is renormalized to Σ = 1.** This is what makes the reactor
//!   total-mass-neutral no matter what numbers the table carries: if lump `i`
//!   supplies mass `f_i`, its row sends `f_i·Σ yields = f_i` back into the slate,
//!   so `Σ products = Σ f_i = 1`. Per-COMPONENT mass is moved (that is the whole
//!   point of a reactor, and why it breaks I7); TOTAL mass cannot be.
//! - **The last band is open-topped** (`upper = None`), so no `t_set` falls off
//!   the top of the table and leaves the feed with nowhere to react.

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::traits::{Reaction, ReactionModel};
use refinery_core::units::{JPerKg, Kelvin, Seconds};

/// One feed lump's outcome within a band: where a unit mass of it goes, and the
/// heat that costs.
#[derive(Debug, Clone)]
pub struct LumpYield {
    /// Product mass distribution over the slate for one unit of this feed lump.
    /// `slate_len` entries; RENORMALIZED to Σ = 1 by `SimpleLookup::new`.
    pub yields: Vec<f64>,
    /// Specific heat of reaction [J/kg of this lump reacting]; positive =
    /// endothermic (absorbed), per `JPerKg`. Weighted by the lump's feed
    /// fraction into the pass's total `Δh_rxn`.
    pub dh_rxn: JPerKg,
}

/// One temperature band: its upper edge and a row per slate lump.
#[derive(Debug, Clone)]
pub struct BandTable {
    /// Upper temperature edge [K]; `None` marks the open-topped last band. A
    /// band is active for `t_set` strictly below `upper` and at/above the
    /// previous band's `upper`.
    pub upper: Option<Kelvin>,
    /// One `LumpYield` per slate lump, indexed by slate position (row `i` is fed
    /// by lump `i`). Length must equal the slate.
    pub rows: Vec<LumpYield>,
}

/// Fixed-conversion reactor (simple fidelity).
pub struct SimpleLookup {
    bands: Vec<BandTable>,
}

impl SimpleLookup {
    /// Build and validate a table over `slate_len` lumps.
    ///
    /// Rejects a table that could not conserve mass or select a band: each band
    /// must carry exactly one row per lump and each row exactly one yield per
    /// lump; finite band edges must strictly ascend; the last band must be
    /// open-topped. Every row's `yields` are renormalized to Σ = 1 (all-zero or
    /// negative rows are refused — there is no meaningful "reacts to nothing").
    ///
    /// # Errors
    /// `SimError::Scenario` on any structural violation, naming what is wrong.
    pub fn new(slate_len: usize, mut bands: Vec<BandTable>) -> Result<Self, SimError> {
        if bands.is_empty() {
            return Err(SimError::Scenario(
                "reactor lookup table has no temperature bands".into(),
            ));
        }
        // Finite edges strictly ascending; only the LAST band may be open-topped.
        let mut prev: Option<f64> = None;
        for (b, band) in bands.iter().enumerate() {
            let is_last = b == bands.len() - 1;
            match band.upper {
                None if !is_last => {
                    return Err(SimError::Scenario(format!(
                        "reactor lookup band {b} is open-topped (upper = None) but is not \
                         the last band; only the highest band may catch all temperatures"
                    )))
                }
                Some(u) => {
                    if !u.value().is_finite() {
                        return Err(SimError::Scenario(format!(
                            "reactor lookup band {b} has a non-finite upper edge"
                        )));
                    }
                    if prev.is_some_and(|p| u.value() <= p) {
                        return Err(SimError::Scenario(format!(
                            "reactor lookup band {b} upper edge {} K does not exceed the \
                             previous band's {} K — bands must strictly ascend",
                            u.value(),
                            prev.unwrap()
                        )));
                    }
                    prev = Some(u.value());
                }
                None => {} // open-topped last band
            }
            if is_last && band.upper.is_some() {
                return Err(SimError::Scenario(
                    "reactor lookup table's last band must be open-topped (upper = None) so \
                     no reactor temperature falls off the top of the table"
                        .into(),
                ));
            }
        }
        // Dimensions + row renormalization.
        for (b, band) in bands.iter_mut().enumerate() {
            if band.rows.len() != slate_len {
                return Err(SimError::Scenario(format!(
                    "reactor lookup band {b} has {} rows, but the slate has {slate_len} lumps \
                     (one row per feed lump is required)",
                    band.rows.len()
                )));
            }
            for (i, row) in band.rows.iter_mut().enumerate() {
                if row.yields.len() != slate_len {
                    return Err(SimError::Scenario(format!(
                        "reactor lookup band {b} row {i} has {} yields, but the slate has \
                         {slate_len} lumps",
                        row.yields.len()
                    )));
                }
                // `Composition::from_weights` is the single owner of "these
                // weights are a valid distribution"; renormalize each row through
                // it so a table with un-normalized rows (Σ ≠ 1) still conserves
                // mass, rather than trusting the author to hand-balance them.
                let normalized = Composition::from_weights(&row.yields).map_err(|e| {
                    SimError::Scenario(format!(
                        "reactor lookup band {b} row {i} is not a valid yield distribution: {e}"
                    ))
                })?;
                row.yields = normalized.fractions().to_vec();
            }
        }
        Ok(Self { bands })
    }

    /// The FCC placeholder table: vacuum gasoil cracks to gasoline, gas and coke;
    /// the products do not crack further at this fidelity. A single open-topped
    /// band (the reactor is isothermal, so temperature-dependence buys nothing
    /// here) resolved against the slate by lump NAME.
    ///
    /// The yields and heat of reaction are ILLUSTRATIVE, not a validated kinetic
    /// set — M4.2's `FourLump` replaces both with Arrhenius rates and lump
    /// formation enthalpies, checked against published yields. Representative FCC
    /// numbers: gasoline ~55 wt% of fresh feed, dry gas/LPG ~15%, coke ~5%, the
    /// rest unconverted cycle oil; overall cracking is endothermic at a few
    /// hundred kJ/kg (Sadeghbeigi, *Fluid Catalytic Cracking Handbook*).
    ///
    /// # Errors
    /// `SimError::Scenario` if the slate lacks any of `gasoil`, `gasoline`,
    /// `gas`, `coke` — the lookup fidelity needs those lumps by name.
    pub fn fcc_demo(slate: &Slate) -> Result<Self, SimError> {
        let idx = |name: &str| -> Result<usize, SimError> {
            slate.index_of(name).ok_or_else(|| {
                SimError::Scenario(format!(
                    "reactions = \"lookup\" (FCC demo) needs a '{name}' lump in the slate; \
                     the demo table cracks gasoil into gasoline, gas and coke"
                ))
            })
        };
        let (gasoil, gasoline, gas, coke) =
            (idx("gasoil")?, idx("gasoline")?, idx("gas")?, idx("coke")?);
        let n = slate.len();

        // Gasoil's single-pass conversion; the rest pass through unchanged.
        let mut gasoil_yields = vec![0.0; n];
        gasoil_yields[gasoil] = 0.25; // unconverted cycle oil
        gasoil_yields[gasoline] = 0.55;
        gasoil_yields[gas] = 0.15;
        gasoil_yields[coke] = 0.05;

        let identity = |i: usize| LumpYield {
            yields: Composition::pure(n, i).fractions().to_vec(),
            dh_rxn: JPerKg::ZERO,
        };
        let mut rows = (0..n).map(identity).collect::<Vec<_>>();
        rows[gasoil] = LumpYield {
            yields: gasoil_yields,
            dh_rxn: JPerKg(350.0e3), // endothermic cracking, per kg of gasoil feed
        };

        Self::new(n, vec![BandTable { upper: None, rows }])
    }

    /// The band active at `temperature`: the first whose upper edge is above it
    /// (finite bands ascend, the last is open-topped, so one always matches).
    fn band(&self, temperature: Kelvin) -> &BandTable {
        self.bands
            .iter()
            .find(|b| match b.upper {
                Some(u) => temperature.value() < u.value(),
                None => true,
            })
            .expect("`new` guarantees an open-topped last band, so one always matches")
    }
}

impl ReactionModel for SimpleLookup {
    fn name(&self) -> &'static str {
        "lookup"
    }

    fn react(
        &self,
        feed: &Composition,
        temperature: Kelvin,
        _tau: Seconds,
        slate: &Slate,
    ) -> Result<Reaction, SimError> {
        let band = self.band(temperature);
        let mut weights = vec![0.0; slate.len()];
        let mut dh_total = 0.0; // Σ f_i · dh_i  [J/kg of feed]
        for (i, &f_i) in feed.fractions().iter().enumerate() {
            let row = &band.rows[i];
            for (w, y) in weights.iter_mut().zip(&row.yields) {
                *w += f_i * y;
            }
            dh_total += f_i * row.dh_rxn.value();
        }
        // Renormalized rows + a normalized feed keep Σ weights = 1, but route the
        // products through the same validator every composition goes through
        // rather than trusting that invariant silently.
        let products = Composition::from_weights(&weights).map_err(|e| {
            SimError::Numerical(format!("reactor products are not a valid composition: {e}"))
        })?;
        Ok(Reaction {
            products,
            dh_rxn: JPerKg(dh_total),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol, Seconds};

    fn slate(names: &[&str]) -> Slate {
        Slate::new(
            names
                .iter()
                .map(|n| PseudoComponent {
                    name: (*n).into(),
                    tb: Kelvin(400.0),
                    molar_mass: KgPerMol(0.1),
                    density: Some(KgPerM3(800.0)),
                    phase: Phase::Liquid,
                    cp: JPerKgK(2000.0),
                })
                .collect(),
        )
        .unwrap()
    }

    /// Feed a pure lump and read back exactly the table row for its band —
    /// per-lump conversion in isolation, the M4.1 gate. Two bands, so band
    /// SELECTION is exercised: the same feed at two temperatures must take two
    /// different rows.
    #[test]
    fn a_pure_lump_converts_to_its_bands_row() {
        let s = slate(&["a", "b"]); // 2 lumps
                                    // Below 700 K: a → 90% a / 10% b. At/above: a → 20% a / 80% b.
        let table = SimpleLookup::new(
            2,
            vec![
                BandTable {
                    upper: Some(Kelvin(700.0)),
                    rows: vec![
                        LumpYield {
                            yields: vec![0.9, 0.1],
                            dh_rxn: JPerKg(1.0e5),
                        },
                        LumpYield {
                            yields: vec![0.0, 1.0],
                            dh_rxn: JPerKg::ZERO,
                        },
                    ],
                },
                BandTable {
                    upper: None,
                    rows: vec![
                        LumpYield {
                            yields: vec![0.2, 0.8],
                            dh_rxn: JPerKg(2.0e5),
                        },
                        LumpYield {
                            yields: vec![0.0, 1.0],
                            dh_rxn: JPerKg::ZERO,
                        },
                    ],
                },
            ],
        )
        .unwrap();

        let pure_a = Composition::pure(2, 0);
        let tau = Seconds(1.0);

        let cool = table.react(&pure_a, Kelvin(500.0), tau, &s).unwrap();
        assert_eq!(cool.products.fractions(), &[0.9, 0.1]);
        assert_eq!(cool.dh_rxn.value(), 1.0e5);

        let hot = table.react(&pure_a, Kelvin(900.0), tau, &s).unwrap();
        assert_eq!(hot.products.fractions(), &[0.2, 0.8]);
        assert_eq!(hot.dh_rxn.value(), 2.0e5);
    }

    /// The reactor is total-mass-neutral for ANY feed and ANY (even
    /// un-normalized) table, because `new` renormalizes every row — the
    /// load-bearing normalization. Here a deliberately un-normalized row (Σ = 2)
    /// still yields products summing to 1.
    #[test]
    fn rows_are_renormalized_so_mass_is_conserved() {
        let s = slate(&["a", "b", "c"]);
        let table = SimpleLookup::new(
            3,
            vec![BandTable {
                upper: None,
                rows: vec![
                    // Σ = 2.0 — must be renormalized to a valid distribution.
                    LumpYield {
                        yields: vec![0.4, 1.0, 0.6],
                        dh_rxn: JPerKg::ZERO,
                    },
                    LumpYield {
                        yields: vec![0.0, 1.0, 0.0],
                        dh_rxn: JPerKg::ZERO,
                    },
                    LumpYield {
                        yields: vec![0.0, 0.0, 1.0],
                        dh_rxn: JPerKg::ZERO,
                    },
                ],
            }],
        )
        .unwrap();

        let feed = Composition::from_weights(&[0.5, 0.3, 0.2]).unwrap();
        let out = table.react(&feed, Kelvin(800.0), Seconds(1.0), &s).unwrap();
        let total: f64 = out.products.fractions().iter().sum();
        assert!(
            (total - 1.0).abs() < 1e-12,
            "products must sum to 1 (mass conservation), got {total}"
        );
    }

    #[test]
    fn fcc_demo_needs_its_named_lumps() {
        let missing = slate(&["gasoil", "gasoline", "gas"]); // no coke
        assert!(SimpleLookup::fcc_demo(&missing).is_err());

        let full = slate(&["gasoil", "gasoline", "gas", "coke"]);
        let demo = SimpleLookup::fcc_demo(&full).unwrap();
        // Pure gasoil cracks to the demo row; products still sum to 1.
        let out = demo
            .react(&Composition::pure(4, 0), Kelvin(800.0), Seconds(2.0), &full)
            .unwrap();
        let total: f64 = out.products.fractions().iter().sum();
        assert!((total - 1.0).abs() < 1e-12);
        assert!(out.dh_rxn.value() > 0.0, "cracking is endothermic");
    }
}
