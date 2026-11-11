# M75.1 — the condenser as a limit (2026-11-11)

**Landed** (DESIGN §80.2, ROADMAP M75.1). The work-in-progress patch kept
here while the slice was paused is now in the tree and has been removed.

## What the pause had read wrong

The paused notes named two suspects: a pressure that did not reach the next
tick, and a condenser duty (~3 945 MW) that looked like the wrong quantity.
Both were one bug. Read every tick, the stored pressure rang between the set
pressure (1.8 bar) and a value growing ~10× each pair of ticks, up to the
design clamp (3.52 bar). The probe printed every 100th tick — always an even
tick, always the set-pressure half — so it read "1.8 bar forever", and the
3 945 MW was the flash of a tick dropped from 3.52 bar back to 1.8.

The cause: the pressure update was implicit in the condenser's slope only.
The column's own answer to its pressure (raise it and the stages stop
boiling, the duty falls) was counted a tick late, with a loop gain near 10.

## The fix

The top pressure is an unknown of the tick, solved by regula falsi
(Illinois) between the set and the design pressure, each trial a whole
column step from the same start (`heat_balance::stepped_pass`,
`condenser_balance`, `illinois`). A column at its set pressure whose
condenser keeps up takes one step, as before.

Lesson kept: sample a suspected oscillation every tick, not every Nth.
