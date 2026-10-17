# M63.4 probe — a column fed more and more water (2026-10-17)

Docs: `docs/DESIGN.md` §68.4; ledger rows C13 (closed by M63.4) and C14 (new).

## Running the two column probes

Neither file is in a test suite. To run one, append its module to
`crates/solvers/tests/suite.rs` for the run, then remove the line again:

```
#[path = "../../../tools/steptime/prototypes/m63_4/probe_crude_column.rs"]
mod c13_probe_crude;
```

```
cargo test -p refinery-solvers --test suite c13_probe -- --nocapture
```

On Windows, wrap it at below-normal priority, as the project guide says.
`probe_test_column.rs` is M62.5's eight-stage column (`reference/wet_column.rs`'s
slate): 8 stages, feed on 5, reflux 2, distillate 0.3, a side draw of 0.25 on
stage 3. `probe_crude_column.rs` is the crude unit's own column: 10 stages,
feed on 8, reflux 4, draws 0.22, 0.16, 0.20 and 0.14 on stages 0, 3, 5 and 7, the
water draw, pumparounds of 15, 15 and 10 MW on stages 4, 6 and 7, at 1.8 bar.
Its oil is held at the wet plant's settled 112.5 kg/s with water added on top,
at the 351 °C liquid-equivalent feed the engine hands the column (and at a
fixed feed enthalpy). Each water share is solved cold (`seed: None`) and warm.

## Results

1. **The test column** separates to 16 % water by mass and refuses from 18 %
   (1 atm) or 20 % (1.8 bar), at feeds of 200, 230 and 300 °C, cold and warm
   alike, all the way to 99.99 %. At 100 % the feed is all water, which §67.17
   rule 10 answers.
2. **The crude unit's column** separates to 40 % water and refuses from 50 %,
   cold and warm alike, at a fixed feed temperature and at a fixed feed
   enthalpy. Its top cools from 166 °C at 0.3 % to 110 °C at 40 %.
3. **The shipped wet crude unit with a wetter crude** halts on tick 1 from 2 %
   water; 1 % runs. Recipe: in `scenarios/crude_unit_wet.toml`, replace the
   crude supply's composition with the oil scaled to `1 − w` plus `water = w`.
   The column's feed falls from about 500 to 355 mol/s at 2 % and 188 mol/s at
   10 %: the supply boils harder, so the charge falls. The cold warm-up then
   refuses: "the draws above the reboiler carry more than the feed". Point 2
   shows this is not the water in the column; it is ledger row C14.
4. **The tank story** (now inline in
   `crates/scenarios/tests/water_flooded_column_reference.rs`): with 4 % water
   in the tank, the slug's last tick is 14 % water and is separated. With
   4.002, 4.004 and 4.006 % it is 34, 51 and 67 % water; before M63.4 each was
   refused and halted the run, and since M63.4 each floods for that one tick.
