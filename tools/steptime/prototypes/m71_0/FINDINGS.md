# M71.0 — a column's draws guessed with its feed (C20), 2026-10-20

Asked as "Yes, as M71" on guessing a column's draws along with its feed: the
fix §75.6 named for C20's lag and the settled stripped plant's 3.5 %. DESIGN
§76.0.

**The probe**: `probe_draws.patch` (on b882379, the M70.3 counting patch
`../m70_3/trace_on_1abbc0b.patch` applied under it). `Engine::predicted_states`
moves each draw of each not-held column (`NodeStates::column_separation`) on as
a node is moved: its temperature by `moved_on`, its composition by
`composition_moved_on`, the parabola from the same ticks (`M71_DRAWS=1`). Two
more switches: `M71_SINCE=1` guesses the draws only from the tick after a
command's (the line from one tick past the command, as the parabola from two);
`M71_FALLBACK=1` solves a guessed tick's first solve again from the last tick's
states when the guessed one fails. `driver_main.rs` is the M70.3 counting
driver; `runs.sh` the four counted stories; `probe_gates.sh` the tests and the
corpus with the guess on.

## Findings

1. **The work comes back** (line-flash flashes a tick, counted, the same every
   round): the settled stripped plant 214.0 → 209.4 (206.7 before M70.1); the
   stripped plant in the 600 ticks after 58 → 65 MW 253.0 → 248.7; the cold
   start-up after tick 700 252.2 → 244.8; the boiling crude unit held 209.8 →
   209.4. Solves and iterations a tick unchanged in every story.
   The rest of the gap to 206.7 is the four draw pumps still: the flow solver's
   instance asks 108.5 questions a tick against 105.8 before M70.1 (over ticks
   1 001–2 500), all at the draw pumps — on some ticks a pump's state and its
   draw differ in their last bits (before M70.1 too, on about a third of
   ticks), and the guess carries the difference on.

2. **The lag closes**: the product line's march at THIS tick's draws (the gate's
   premise before M70.1) holds at its 1e-9 with the draws guessed and fails
   without (§75.3's restatement reads the last tick's).

3. **M70.1's moves after a key press were mostly this lag.** Against the engine
   before M70.1 (0335994), the largest relative move: the stripped plant
   58 → 45, 65, 75 MW 3.3e-7, 1.4e-7, 3.2e-7 (M70.1: 4.7e-6, 4.3e-7, 1.0e-6);
   the cold start-up 1.1e-6 (M70.1: 1.6e-5); the boiling crude unit 58 → 65 MW
   1.5e-8 (2.5e-7). The heavy draws' pump and cooler pressures that led M70.1's
   moves go back to within ~1e-8 of where they stood (the cold start-up's residue cooler 635 606.1088
   against 635 606.1019 Pa before M70.1; M70.1 had it at 635 595.81).

4. **The corpus**: 6 of 67 plants move on each fidelity, the six coil crude
   units, their iterations unchanged to the count; none fails.

5. **A draw guessed across a command can break a run.** With `M71_DRAWS` alone,
   `wetter_crude_reference::slow_a_wet_slug_floods_the_column_and_passes` fails:
   on the second tick after the crude is stepped back from the 50 % slug, the
   straight line from the flooded column's draws to the recovering ones guesses
   a state from which the kerosene line "arrives at −536 748 bar", and the solve
   refuses. Either switch alone mends it: `M71_SINCE` (no draw guess across the
   command), or `M71_FALLBACK` (it fires once, tick 502, and the tick solves
   unguessed). A column that floods or recovers with no command is covered only
   by the fallback.

6. **Tests**: with `M71_DRAWS` and `M71_SINCE` the FULL tier fails only
   `product_line_reference` (finding 2: its §75.3 restatement no longer holds;
   its original premise does).

7. **The fallback alone** (`M71_DRAWS` + `M71_FALLBACK`, no `M71_SINCE`): over
   the FULL tier and both corpora it fires once — the slug test's tick 502 —
   and the corpus is byte-identical to `M71_SINCE`'s on both fidelities. (The
   FULL tier's one failure there, `density_memo_tests`, is the M70.3 counting
   patch reading a test's one-word key, not the engine.) With `M71_SINCE`
   nothing in the tests or the corpus reaches a fallback: no solve failed.

8. **What a fallback would retry is clean**: both flow solvers run one driver
   (`solve_remembering_reliefs` → `solve_with_active_anchoring_with`), which
   commits the warm start only from a converged pass and returns a failure
   before writing it; the retry's `begin_solve` turns the solver's
   `DensityMemo` once more, which, keyed on every input, changes no answer.

## To re-run

The scripts name `W:/temp/claude/m71_0/` (worktrees `wt`, `base`, `pre` of
b882379, b882379 and 0335994 with the counting patches; the drivers' builds;
the output folders), removed at M71's close. Rebuild: the worktrees with their
patches, `driver_main.rs` built `--release` against each.
