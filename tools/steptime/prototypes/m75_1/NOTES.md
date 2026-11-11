# M75.1 — the condenser as a limit, work in progress (2026-11-11)

Stopped mid-slice at the user's "wrap up". **Not committed as engine code**;
`wip.patch` is the whole change on top of `d9099a1`. To resume, run
`git apply tools/steptime/prototypes/m75_1/wip.patch`.

## What the patch does (DESIGN §80.1)

- Core:
  - `ColumnCondenser` (margin, coolant, overhead volume, set pressure, design
    pressure) on `ColumnHoldup::condenser`.
  - `HoldupState::column_pressure` and `condenser_ua`, absent and not
    serialized without a condenser.
  - `running_column_pressure()`, which the three readers of the column's
    pressure now use: the network pin (`network.rs`), the draw lines' start
    (`line_stage_pressure`) and the column pass (`energy.rs`).
  - `CondenserReading`, shown in the snapshot as `HoldupOperation::condenser`.
- Solvers (`heat_balance.rs`):
  - `condenser_ua` sizes the condenser on the first tick: `UA = Q₀·(1+margin)/(T_b − T_coolant)`.
  - `next_column_pressure` is a linearly implicit step of
    `dP/dt = (R·T/V)(Q − C(P))/λ`, clamped between the set and design
    pressures. It runs after each step in `stepped_pass`.
- Scenarios:
  - A new cascade key, `condenser_limit = { margin, coolant_c, overhead_m3 }`.
    The name `condenser` is taken by "total"/"partial".
  - Refusals: declared without holdup, a negative margin, a bad coolant
    temperature, an overhead volume not above zero.
  - The design pressure is a game value: the larger of 1.1 × the set
    pressure and the set pressure + 1.72 bar.
- Tests: `column_condenser.rs`, with only a probe test so far. The
  `column_empty.rs` helpers were made `pub(crate)` for it.

It builds with no warnings. Nothing else has been run: no FAST tier, no
clippy, no corpus.

## The open problem, first thing next session

The probe: the holdup plant with its steel and `margin = 0.05, coolant_c =
35, overhead_m3 = 100`, settled for 100 ticks, total reflux on, 300 s.

- `UA` came out at 0.58 MW/K.
- At 0 s the reading said the capacity was 47.4 MW, the duty 48.3 MW and the
  condenser at capacity, yet **the pressure stayed at exactly 1.8 bar for all
  300 s**.
- The snapshot's `column_duty.condenser_w` reads **~3 945 MW** after total
  reflux goes on. That is about 80 times M75.0's 53.6 MW at one minute.
- The books still close: worst 9e-8 kg/s and 3.7 W.

Two suspects:

1. The pressure written to `next.column_pressure` doesn't reach the next
   tick. Possibly the engine's write-back, the drop re-solves, or the
   start-empty path drops it. Check `state(&engine).column_pressure` after a
   tick.
2. The duty passed in (`stepped.condenser_duty`) isn't the condenser's.
   3 945 MW looks like a different quantity, or a reading bug on total reflux
   that M75.0's numbers didn't show. Compare it with M75.0's `overfill.txt`.

## Still to do in M75.1

- Fix the above.
- Tests:
  - the key read and refused;
  - at rest, flows bit-identical to the same plant without the limit, and the
    pressure exactly the set pressure;
  - `UA` checked by hand;
  - on total reflux, the pressure rising with the books closed.
- The FAST and FULL tiers, clippy, fmt, and the corpus on both fidelities
  (every plant should stay byte-identical).
- DESIGN §80.1 to note the implicit balance in place of the lag.
