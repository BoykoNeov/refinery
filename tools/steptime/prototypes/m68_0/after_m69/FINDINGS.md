# M68.0's probe re-run after M69 (C11's trigger), 2026-10-19

C11's row said: re-run this probe after A29. M69 took A29 (the line flash
remembered for a tick, a drifting tick's first solve predicted), so this is
that re-run. Nothing in the engine changed for it.

**How.** `../probe_count.patch` applied unchanged on ed5fa07 (M69's last
commit) in a temporary worktree; the column files it touches have not changed
since its base, 58ffdc1. `cargo build --release` of `m68_0_count`, run over
the nine column plants with `DUMP` (`count.txt`), then the stripped, boiling
and cold plants three more times. `minof.py` keeps each tick's fastest run
and `windows.py` summarises windows of the story, each starting on its
command's tick, and `flip.py` counts the ticks over 2.5 ms that would come
inside it without the Jacobian or the column (`windows_fastest_of_3.txt`).
The machine was 19–42 % busy with other work, so the counts decide and the
times are the fastest of three, upper bounds.

**The probe is the same probe.** `crude_unit` did not move at M69 (its corpus
fingerprint is unchanged), and its counts match M68's to the digit: 25.6
evaluations a tick, median 25, max 758, at most 21 iterations. So do the
hot-fed and overheated columns'.

## Findings

1. **The column's repeat solves are gone with A29's.** Column solves a tick,
   M68 → now: stripped 3.37 → 1.56, cold 4.78 → 2.04, boiling 2.00 → 1.00,
   wet and steamed 1.57 → 1.00. Evaluations a tick: stripped 61.5 → 19.7
   (median 105 → 27), cold 107.2 → 36.1, boiling 11.4 → 5.6, wet and steamed
   7.0 → 4.2. The worst ticks are unchanged (stripped 256, cold 231): those
   are command ticks (the stripped plant's steam valve at tick 1901, the cold
   start-up's first reboiler step-down that bites, tick 730), where M69's
   predictor does not act.

2. **The column is still not the cost on any furnace plant.** It is 4–12 % of
   their tick (`separate (all)`): 7 % on the stripped plant, 9 % on the cold
   start-up, 7 % on the boiling plant, 4 % on the wet and steamed ones. With
   the column made free, no tick far past 2.5 ms comes inside it: after a
   furnace step and through the cold start-up, not one (`flip.py`, which
   takes off the Jacobian's time, the part C11 would cut, and as an upper
   bound the whole column's). The ticks AT the line are another matter: the
   settled stripped plant runs a median 2.47 ms and the held boiling plant
   2.34 ms, and of their ticks over 2.5 ms, 422 of 477 and 56 of 790 would
   come inside without the Jacobian. On a machine this busy, and in a build
   faster than the screen's, those ticks sit on the line; this run cannot
   say which side of it they are on.

3. **The stripped plant after a furnace step, measured** (M69 left it
   estimated at 8–9 ms). Fastest of three, per tick, in the probe's release
   build: settled before any step, median 2.5 ms; for the 600 ticks after the
   furnace goes 20 → 45 MW, median 6.2 ms, 7.7 ms over the first 60 ticks,
   every tick over 2.5 ms, the worst 19 ms; after the 58 MW step, the same.
   The engine solves the plant about twice a tick for the whole minute (the
   coil still drifting), and the column is 7–8 % of it. The screen's `godot`
   build is optimised less than this release build, so on the screen it would
   not be faster.

4. **The cold start-up**: median 6–7 ms a tick through all 2 400 ticks, every
   one over 2.5 ms, about two solves a tick; the column 8–12 %.

5. **The crude unit's big key press is unchanged**: one tick of 758
   evaluations (~8 ms by M68's ~11 µs an evaluation), then 25 again. This is
   still the only place C11's trigger ("a column whose solve is the frame's
   cost") is met.

## So

C11 is where M68 left it: a cheaper Jacobian would save at most the column's
4–12 % on the furnace plants, and would move every column plant and re-open
the screen tables (C19). What fills the furnace plants' ticks is the coil
still drifting after a step: about two solves a tick for at least a minute,
6–8 ms each. That is A29's leftover, now measured rather than estimated, and
it is the bigger cost.

## To re-apply

`git worktree add --detach <dir> ed5fa07`, `git -C <dir> apply
../probe_count.patch`, then `cargo build --release --manifest-path
<dir>/tools/steptime/Cargo.toml --bin m68_0_count` and run
`<dir>/tools/steptime/target/release/m68_0_count <dir>/scenarios [plant ...]`
with `DUMP=<out dir>`. The worktree used here (`W:/temp/claude/refinery-m70-0`)
and its output folder (`W:/temp/claude/m70-0-out`) were removed afterwards.
