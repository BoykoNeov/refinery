# Plans

One file per piece of work that came out of the 2026-11-12 review
(`docs/GAME.md` says why each matters). Each is written to be carried out by
a smaller model than the one that wrote it, in a fresh session, with no
memory of the review. So each says what to read first, what to change, how
to know it worked, and **where to stop and ask the user**.

None of these is a milestone until the user says so. Taking one means
opening a milestone for it the usual way (CLAUDE.md, "Current milestone").

## Order

Recommended, cheapest and most useful first. The first three need no engine
change and no decision.

| # | plan | touches | size | needs a decision first? |
|---|---|---|---|---|
| 1 | [small-fixes.md](small-fixes.md) | `cli`, docs | an hour | no |
| 2 | [refusal-messages.md](refusal-messages.md) | `core` messages, tests | half a day | no |
| 3 | [failed-tick-recovery.md](failed-tick-recovery.md) | `godot-ext` bridge, screens | slice 1 a day; slice 2 larger | slice 2 yes |
| 4 | [save-and-load.md](save-and-load.md) | `godot-ext` bridge, menu | a day after #3 | file location only |
| 5 | [plant-map-screen.md](plant-map-screen.md) | `demo/map.gd` | a day per slice | slice 4 yes |
| 6 | [screen-kit.md](screen-kit.md) | `demo/*.gd` | two days | no |
| 7 | [objectives-and-alarms.md](objectives-and-alarms.md) | everything | — | **yes, before anything** |

## Rules for whoever carries one out

Read `CLAUDE.md` first; it overrides anything here. On top of it:

1. **One plan, one slice, one commit.** Do not combine plans. Do not tidy
   what the plan does not name.
2. **Every existing plant must stay byte-identical** unless the plan says a
   slice moves it. Any commit touching `crates/*/src` runs the FULL tier and
   the corpus on both solvers against the rolling baseline (CLAUDE.md,
   "Which tests run when"). If a plant moves and the plan did not predict
   it, stop and report; do not refresh the baseline.
3. **Never edit an existing file under `scenarios/`.**
4. **Tests first** for anything in Rust: write the failing test, see it
   fail, then the change.
5. **Godot work is verified by running it**, not by reading it. Every screen
   has a headless `--auto` run and a windowed `--shots=<dir>` run
   (CLAUDE.md lists them); `demo/map.tscn -- --auto` loads all 71 plants.
   Look at the screenshots. Temp output goes under `W:\temp\claude`.
6. **GDScript files use TABS**, and `demo/pump.gd` and `demo/plant.gd` have
   Windows line endings — keep each file's own.
7. **A fork the plan marks "ask" is asked**, in plain words, with the
   options, what each leads to, and a recommendation. Then wait.
8. When a slice lands, tick it in the plan's own checklist and say in the
   commit which plan and slice it was.
