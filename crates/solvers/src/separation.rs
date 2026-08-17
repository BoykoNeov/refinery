//! Separation fidelities: how a column divides its feed among its draws.
//!
//! `CutPointSplitter` is M3.2's boiling-range splitter, moved here **verbatim**
//! from `core::energy::column_separation` when M7.1 put the `SeparationModel`
//! seam under it. The M7.3 stage cascade lands beside it, behind the same trait.

use refinery_core::components::Composition;
use refinery_core::error::SimError;
use refinery_core::traits::{ColumnPass, DrawSeparation, Separation, SeparationModel, ThermoModel};

/// Split a column feed into its draws by boiling range (DESIGN §5) — the simple
/// (game) separation fidelity, and the default every pre-M7 scenario means.
///
/// For each draw `i` a component `c` gets a weight `w_ic` from a ramp across the
/// cut points; the split and per-draw composition are
///
/// ```text
/// splitᵢ    = Σ_c f_feed,c · w_ic
/// comp_i,c  = f_feed,c · w_ic / splitᵢ
/// ```
///
/// **`Σᵢ w_ic = 1` holds by construction, not by luck.** Writing draw `i`'s
/// weight as a difference of cumulative "fraction below cut" terms telescopes:
/// `w_0 = 1 − h₀`, `w_i = h_{i−1} − h_i`, `w_{last} = h_{last−1}`, summing to 1
/// for every component. With cut temperatures strictly increasing, each `h_{i−1}
/// ≥ h_i` pointwise, so every weight is non-negative without a clamp. The splits
/// are then normalized defensively so `Σᵢ splitᵢ = 1` to machine precision — the
/// identity that makes the column mass-neutral and (via M3.1's linearity, since
/// `Σᵢ splitᵢ·cpᵢ = cp_feed`) energy-neutral both.
///
/// A draw whose band catches no feed component has `splitᵢ = 0`; it carries the
/// feed composition as an inert placeholder (its flow will be zero), never a
/// divide-by-zero.
///
/// **What it ignores, and why that is the design.** The feed *flow*, the column
/// *pressure* and the `ThermoModel` are all in `ColumnPass` and all unread here:
/// a boiling-range split is a property of the feed's composition alone. Each is
/// load-bearing for the cascade, and carrying them from M7.1 is the `tau`
/// precedent — the swap costs no trait churn (DESIGN §5, fork 2 + correction 2).
pub struct CutPointSplitter;

impl SeparationModel for CutPointSplitter {
    fn name(&self) -> &'static str {
        "cut_point"
    }

    fn separate(
        &self,
        pass: &ColumnPass<'_>,
        _thermo: &dyn ThermoModel,
    ) -> Result<Separation, SimError> {
        let draws = pass.draws;
        let slate = pass.slate;
        let n = draws.len();
        let s = pass.smearing.value();
        let f = pass.feed.fractions();

        // Cumulative "fraction of a component at or below draw i's upper cut". The
        // heaviest draw (`upper_cut = None`) is the open catch-all: everything is at
        // or below +∞, so its cumulative is 1.
        let cumulative_below = |i: usize, tb: f64| -> f64 {
            match draws[i].upper_cut {
                Some(cut) => 1.0 - cut_fraction_above(tb, cut.value(), s),
                None => 1.0,
            }
        };

        let mut result: Vec<DrawSeparation> = Vec::with_capacity(n);
        for i in 0..n {
            let mut weights = vec![0.0; slate.len()];
            let mut split = 0.0;
            for (c, &fc) in f.iter().enumerate() {
                let tb = slate.get(c).tb.value();
                let c_i = cumulative_below(i, tb);
                let c_prev = if i == 0 {
                    0.0
                } else {
                    cumulative_below(i - 1, tb)
                };
                let w_ic = c_i - c_prev; // ≥ 0 since cuts increase ⇒ c_i ≥ c_prev
                let contribution = fc * w_ic;
                weights[c] = contribution;
                split += contribution;
            }
            let composition = if split > 0.0 {
                Composition::from_weights(&weights).map_err(|e| {
                    SimError::Numerical(format!(
                        "column draw {i} produced no valid composition: {e}"
                    ))
                })?
            } else {
                // Inert draw: no feed component in its band. Its flow is zero, so the
                // placeholder is never carried anywhere; the feed keeps it finite.
                pass.feed.clone()
            };
            result.push(DrawSeparation {
                split,
                composition,
                // Every draw leaves at the feed temperature. Not a placeholder —
                // it is what a fixed-cut splitter means, and it matches what the
                // engine already does (a column is a swept zero-volume node whose
                // draws read it as their upwind end), which is why M7.1 stays
                // bit-identical while the field exists. A cascade's draws leave at
                // their own tray temperatures instead.
                temperature: pass.temperature,
            });
        }

        // Defensive normalization: the telescoping sum is 1 in exact arithmetic, so
        // this only removes float drift, but it is what the DESIGN note names as the
        // mechanism that *enforces* `Σ splitᵢ = 1` rather than hoping for it. Guarded
        // against an all-zero feed band that no valid composition can produce.
        let total: f64 = result.iter().map(|d| d.split).sum();
        if total > 0.0 {
            for d in &mut result {
                d.split /= total;
            }
        }

        Ok(Separation {
            draws: result,
            // A fixed-cut split has no condenser and no reboiler: it is a
            // stoichiometric bookkeeping rule, not an energy-driven separation.
            // `None` rather than the `Watt::ZERO` this returned through M7.3 —
            // the two are not the same claim, and once the cascade started
            // computing real duties the zero would have been the only number a
            // frontend ever saw for a cut-point column. There is no duty here to
            // be zero; the column's ONLY energy statement at this fidelity is the
            // one M3.2 proved, that `Σᵢ splitᵢ·cpᵢ = cp_feed` makes it
            // energy-neutral, and that needs no duty to hold.
            condenser_duty: None,
            reboiler_duty: None,
        })
    }
}

/// The mass fraction of a component boiling at `tb` that lands on the HEAVY side
/// of a cut point at `cut` [K], given a `smearing` ramp width [K].
///
/// A linear ramp centred on the cut: `0` a half-width below, `0.5` exactly at the
/// cut, `1` a half-width above. `smearing = 0` is the sharp splitter — a step,
/// with a component boiling exactly on the boundary shared evenly. The `s <= 0`
/// branch also guards the `(tb − cut)/s` divide that the sharp case would hit.
fn cut_fraction_above(tb: f64, cut: f64, smearing: f64) -> f64 {
    if smearing <= 0.0 {
        if tb < cut {
            0.0
        } else if tb > cut {
            1.0
        } else {
            0.5
        }
    } else {
        ((tb - cut) / smearing + 0.5).clamp(0.0, 1.0)
    }
}

/// The cut-point splitter's math, in isolation, against hand calculations. This
/// is the gate the DESIGN note names as the ONLY one with discriminating power
/// over a column: a splitter conserves every component identically, so I7 is
/// green by construction and cannot see a cut boundary off by one or a disabled
/// smearing — but a per-draw *composition* vector against a hand calc can.
///
/// Moved here with the code in M7.1, unchanged except for the call shape.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::ConstantThermo;
    use refinery_core::components::{Phase, PseudoComponent, Slate};
    use refinery_core::graph::{ColumnDraw, NodeId};
    use refinery_core::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol, KgPerSec, Pascal};

    /// A slate whose only meaningful property here is each cut's boiling
    /// point — density/cp/MW are placeholders, since separation reads only
    /// `tb`.
    fn slate_with_tbs(tbs: &[f64]) -> Slate {
        Slate::new(
            tbs.iter()
                .enumerate()
                .map(|(i, &tb)| PseudoComponent {
                    name: format!("cut{i}"),
                    tb: Kelvin(tb),
                    molar_mass: KgPerMol(0.1),
                    density: Some(KgPerM3(800.0)),
                    cp: JPerKgK(2000.0),
                    phase: Phase::Liquid,
                })
                .collect(),
        )
        .unwrap()
    }

    /// The outlet id is unused by the splitter (it splits by boiling range, not
    /// by which tank a draw feeds), so a dummy id suffices.
    fn draw(upper_cut_k: Option<f64>) -> ColumnDraw {
        ColumnDraw::by_cut(NodeId(0), upper_cut_k.map(Kelvin))
    }

    /// The feed state the splitter ignores, held constant across these tests so
    /// a difference in a result can only come from the cut structure.
    const FEED_T: Kelvin = Kelvin(500.0);

    fn split(
        slate: &Slate,
        feed: &Composition,
        draws: &[ColumnDraw],
        smearing: Kelvin,
    ) -> Vec<DrawSeparation> {
        CutPointSplitter
            .separate(
                &ColumnPass {
                    slate,
                    draws,
                    smearing,
                    pressure: Pascal(160_000.0),
                    feed,
                    feed_flow: KgPerSec(7.0),
                    temperature: FEED_T,
                    cascade: None,
                },
                &ConstantThermo,
            )
            .unwrap()
            .draws
    }

    /// SHARP splitter (smearing 0): each component lands wholly in the one
    /// band its boiling point falls in. Four cuts at 50/150/250/350 K, three
    /// draws split at 100 and 300 K, feed [0.1, 0.2, 0.3, 0.4]:
    ///
    ///   draw 0 (< 100): cut0 only            → split 0.1, comp [1,0,0,0]
    ///   draw 1 (100–300): cut1, cut2         → split 0.5, comp [0,0.4,0.6,0]
    ///   draw 2 (> 300): cut3 only            → split 0.4, comp [0,0,0,1]
    ///
    /// THE MUTATION THIS EXISTS FOR: a cut boundary off by one component
    /// moves cut2 (250 K) from draw 1 into draw 0, which no mass balance sees
    /// — total in still equals total out — but which this per-draw comp
    /// assertion fails loudly on.
    #[test]
    fn sharp_splitter_assigns_each_cut_to_its_band() {
        let slate = slate_with_tbs(&[50.0, 150.0, 250.0, 350.0]);
        let feed = Composition::from_weights(&[0.1, 0.2, 0.3, 0.4]).unwrap();
        let draws = [draw(Some(100.0)), draw(Some(300.0)), draw(None)];

        let sep = split(&slate, &feed, &draws, Kelvin(0.0));

        let splits: Vec<f64> = sep.iter().map(|d| d.split).collect();
        assert!(
            (splits[0] - 0.1).abs() < 1e-12
                && (splits[1] - 0.5).abs() < 1e-12
                && (splits[2] - 0.4).abs() < 1e-12,
            "splits should be [0.1, 0.5, 0.4], got {splits:?}"
        );
        let expect = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.4, 0.6, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        for (i, want) in expect.iter().enumerate() {
            for (c, w) in want.iter().enumerate() {
                assert!(
                    (sep[i].composition.fractions()[c] - w).abs() < 1e-12,
                    "draw {i} comp[{c}] should be {w}, got {}",
                    sep[i].composition.fractions()[c]
                );
            }
        }
    }

    /// SMEARED splitter: a component boiling INSIDE the ramp of a cut is split
    /// between the two adjacent draws. Two cuts at 175/400 K, one boundary at
    /// 200 K, smearing 100 K (half-width 50, band 150–250):
    ///
    ///   cut0 (175 K): fraction heavy = (175−200)/100 + 0.5 = 0.25
    ///                 → 0.75 to draw 0, 0.25 to draw 1
    ///   cut1 (400 K): far above       → 1.0 to draw 1
    ///
    /// Feed [0.5, 0.5]:
    ///   split 0 = 0.5·0.75            = 0.375, comp [1, 0]
    ///   split 1 = 0.5·0.25 + 0.5·1.0  = 0.625, comp [0.2, 0.8]
    ///
    /// THE MUTATION THIS EXISTS FOR: smearing silently disabled (treated as
    /// 0). cut0 would then land WHOLLY in draw 0 — split 0.5, draw 1 comp
    /// [0, 1] — so this assertion, unlike the sharp one, cannot pass unless
    /// the ramp is actually applied. The sharp test alone leaves smearing
    /// untested (its answer is the same at smearing 0), which is why both
    /// exist.
    #[test]
    fn smearing_splits_a_boundary_cut_between_adjacent_draws() {
        let slate = slate_with_tbs(&[175.0, 400.0]);
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let draws = [draw(Some(200.0)), draw(None)];

        let sep = split(&slate, &feed, &draws, Kelvin(100.0));

        assert!(
            (sep[0].split - 0.375).abs() < 1e-12 && (sep[1].split - 0.625).abs() < 1e-12,
            "smeared splits should be [0.375, 0.625], got [{}, {}]",
            sep[0].split,
            sep[1].split
        );
        let d1 = sep[1].composition.fractions();
        assert!(
            (d1[0] - 0.2).abs() < 1e-12 && (d1[1] - 0.8).abs() < 1e-12,
            "draw 1 (with smearing) should be [0.2, 0.8], got {d1:?}; a sharp \
             splitter would give [0, 1] — this is what a disabled smearing fails"
        );
    }

    /// `Σᵢ splitᵢ = 1` for any feed and any cut structure — the identity that
    /// makes the column mass-neutral (and, via linearity, energy-neutral). It
    /// holds by telescoping before the defensive normalization, so this is a
    /// tight bound. A five-cut feed through four draws with a nontrivial
    /// smearing exercises overlapping ramps.
    #[test]
    fn splits_always_sum_to_one() {
        let slate = slate_with_tbs(&[60.0, 140.0, 210.0, 300.0, 420.0]);
        let feed = Composition::from_weights(&[0.05, 0.30, 0.15, 0.10, 0.40]).unwrap();
        let draws = [
            draw(Some(120.0)),
            draw(Some(250.0)),
            draw(Some(360.0)),
            draw(None),
        ];

        let sep = split(&slate, &feed, &draws, Kelvin(40.0));
        let total: f64 = sep.iter().map(|d| d.split).sum();
        assert!(
            (total - 1.0).abs() < 1e-12,
            "Σ splitᵢ must be 1 (mass neutrality), got {total}"
        );
    }

    /// A draw whose band catches NO feed component has split 0 and carries the
    /// feed composition as an inert placeholder — never a divide-by-zero. Here
    /// the middle draw's band (150–250 K) contains neither cut (100 K, 400 K).
    #[test]
    fn an_empty_band_draw_is_zero_split_and_carries_the_feed() {
        let slate = slate_with_tbs(&[100.0, 400.0]);
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let draws = [draw(Some(150.0)), draw(Some(250.0)), draw(None)];

        let sep = split(&slate, &feed, &draws, Kelvin(0.0));
        assert!(
            sep[1].split.abs() < 1e-12,
            "the empty middle band must take no mass, got split {}",
            sep[1].split
        );
        assert_eq!(
            sep[1].composition.fractions(),
            feed.fractions(),
            "an empty draw carries the feed composition as an inert placeholder"
        );
    }

    /// Every draw leaves at the feed temperature it was HANDED — not at some
    /// value the splitter invents, and not at a constant.
    ///
    /// This pins the contract M7.4 will wire into `edge_temperature_at`'s column
    /// arm. It is deliberately tested HERE, on the impl, rather than through the
    /// engine: the sweep gives a column's draws its mixed feed temperature by the
    /// ordinary upwind rule, so an engine-level assertion would compare that
    /// number against itself and pass no matter what this function returned.
    #[test]
    fn a_draw_leaves_at_the_feed_temperature_it_was_handed() {
        let slate = slate_with_tbs(&[100.0, 400.0]);
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let draws = [draw(Some(250.0)), draw(None)];

        let sep = split(&slate, &feed, &draws, Kelvin(0.0));
        for (i, d) in sep.iter().enumerate() {
            assert_eq!(
                d.temperature, FEED_T,
                "draw {i} must leave at the feed temperature the pass carried"
            );
        }
    }

    /// This fidelity reports **no** duties — `None`, not zero.
    ///
    /// A boiling-range split has no condenser and no reboiler to have a duty, so
    /// the honest answer is the absence of one. Through M7.3 this asserted
    /// `Watt::ZERO`, which was the same assertion the cascade's own stub then
    /// satisfied for an entirely different reason ("M7.4 has not landed"); M7.4b
    /// separates them in the type, so the two fidelities can no longer be
    /// confused by a reader OR by a frontend. `NodeSnapshot::column_duty` omits
    /// the field entirely on this path, which is what keeps the twelve scenarios
    /// byte-identical.
    #[test]
    fn the_splitter_reports_no_condenser_or_reboiler_duty() {
        let slate = slate_with_tbs(&[100.0, 400.0]);
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let draws = [draw(Some(250.0)), draw(None)];

        let sep = CutPointSplitter
            .separate(
                &ColumnPass {
                    slate: &slate,
                    draws: &draws,
                    smearing: Kelvin(0.0),
                    pressure: Pascal(160_000.0),
                    feed: &feed,
                    feed_flow: KgPerSec(7.0),
                    temperature: FEED_T,
                    cascade: None,
                },
                &ConstantThermo,
            )
            .unwrap();

        assert_eq!(sep.condenser_duty, None);
        assert_eq!(sep.reboiler_duty, None);
    }
}
