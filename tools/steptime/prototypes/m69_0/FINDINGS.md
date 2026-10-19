# M69.0 — a coil plant's tick, and a predictor for its re-solves (A29), 2026-10-19

The probe: `probe_predict.patch` (on 14810ee; it also carries M68.0's column
counters) times the engine's tick by part — the flow solve, the resolve, the
coil's cells inside it, the density check that decides a re-solve, the line
flash's `density` and `settle` — counts every line-flash question asked twice
in one tick with exactly the same arguments, logs each re-solve's density
change, and adds a switchable PREDICTOR: `M69_PREDICT=1` reads, for the first
solve only, each zero-volume node's temperature and liquid-equivalent moved on
by the last tick's move (`T + (T − T_prev)`); `=2` skips that on the tick after
a command. The driver (`m69_0_count.rs`): the boiling crude unit held 1 000
ticks then its furnace 20 → 25 MW, the stripped crude unit held 1 000 then
58 → 65 MW (the screen's slow case), the cold start-up (2 400 ticks), the wet
crude unit held; `run.sh <mode>` writes snapshots at eight ticks, and
`compare.py` gives the largest relative difference between two runs' fields.

**The machine was loaded by other work throughout**: counts and shares decide;
milliseconds swung 3–4 times between runs and are not quoted.

## Findings

1. **The line flash is the coil plants' tick.** On the boiling, stripped, cold
   and wet plants `density` and `settle` together take some 80 % of the wall
   (`where_the_time_goes.txt`; the timers nest, so shares do not add); the
   coil's cells are 30–43 % (their fixed point settles each of the eight cells
   about ten times a pass: 82 settles); the column 4–15 %.

2. **The re-solves multiply it.** Passes a tick: the boiling plant 2 held, 4
   after its furnace step; the stripped plant 1.6 held, 5 after its step; the
   cold start-up 4.8 throughout. Each pass's density change falls about 20
   times (1.4e-4, 7.0e-6, 1.3e-7, 5.1e-9 against `SWEPT_DENSITY_TOL` 1e-8) —
   A29's geometric fall — and each pass costs about the same.

3. **An identical-answers memo is bounded at ~15 % of the tick**
   (`repeats_and_passes.txt`): a third of the `density` questions in a tick
   repeat one asked earlier in it with exactly the same arguments (as a
   density or as a settle), but only 5 % of the `settle` questions. Exact keys
   keep the answers bit for bit (M56.0's argument).

4. **The predictor halves the passes** (`predict_mode*.txt`): the boiling plant
   2 → 1 held and 4 → 2 after the step; the stripped plant 1.6 → 1.0 held and
   5 → 2.0 after; the cold start-up 4.8 → 2.0. The first pass's change falls
   from ~1e-4 to ~1e-7 (the drift is smooth: what is left is its curvature).

5. **After a key press it neither overshoots nor helps for two ticks**: on the
   step tick and the one after, the passes are the shipped engine's (the duty
   moves the coil metal over ticks, not the node states at once); from the
   third tick, 2–3 passes. The guard (`=2`) changed only the step tick itself
   and no count that matters here; a command that makes the states jump (a
   valve) is not measured.

6. **The answers move within the solvers' tolerances**
   (`answers_moved_mode*.txt`, the solver's own report left out): at most
   6.5e-7 relative (the cold plant's cut-back gas oil share, 0.1566160 →
   0.1566159), the stripped plant 1.6e-7, the boiling plant 1e-8. Not bit for
   bit: every plant that carries vapour moves its fingerprint, and the
   stripped and cold screen tables (C19) are owed again.

7. **What is left after both**: the passes halved and the memo's ~15 %
   together put a coil plant's tick near 0.35–0.45 of today's after a step —
   the stripped plant's 24–26 ms on the screen near 8–9 ms, still past the
   2.5 ms frame (an estimate from the counts, not measured on the screen). The rest is
   each pass's own cost: the coil's ~82 settles and ~130 density questions.
   Fewer cells (DESIGN §67.14, measured at M62.4c: four cells 5.1 against
   eight's 7.2 ms a tick, the outlet 0.7 K and the metal 4 K under) or a
   smoothed density near the boiling onset change the physics, and were not
   built.

## Checked before the decision (the advisor's list)

8. **Off, the probe is the shipped engine**: the corpus, 6 000 ticks, both
   fidelities, `M69_PREDICT=0` against the rolling baseline (read only, 4
   jobs): 67 of 67 byte-identical on each (`corpus_*_0.txt`).

9. **On (`=2`), 9 of 67 plants move on each fidelity and none fails**
   (`corpus_*_2.txt`): the six coil crude units, `flashing_rundown`,
   `product_cooler`, `wet_rundown` — every plant whose lines carry vapour and
   re-solve on it; `pump_gas_lock` (its lock and vent are not commands, so the
   guard does not cover them) is byte-identical. The corpus does not count
   re-solves, and its wall column under 4 jobs on a loaded machine is noise
   (single plants swung 2×), so neither fidelity's speed is read from it.

10. **The flow solve reads from the states it is handed** (`network.rs`): each
    zero-volume node's liquid-equivalent (or temperature) and composition for
    its densities, the vapour share's `mass_fraction == 1.0` as an all-vapour
    flag, and the column's draws. The predictor moves only the first; the
    other two are what the shipped engine's first solve reads too.

11. **The FULL tier with the predictor on: 1 failure of 738**
    (`tests_predict.txt`): `coil_cell_reference::a_coil_in_cells_closes_its_first_law_every_tick`,
    the coil's books missing by 2.4e-9 of its duty against the gate's 1e-12
    (shipped: 1.3e-14) — on EVERY tick from the third, at 2 re-solves as well
    as at none, so the guess reaches past the first solve. It is the
    liquid-equivalent's guess alone: the temperatures alone pass, and guessing
    the vapour share and latent heat with it changes the miss in no digit. Not
    traced; a built predictor traces it and keeps the gate at 1e-12. No other
    published or gated number moved (the cold start-up's walk to zero
    included).

## Traced while building M69.2

12. **The leak is the flow solver's tolerance** (`coil_cell_reference`
    printing each tick's re-solves, residual and coil flow in − out): with the
    predictor the last re-solve starts inside the Newton tolerance and is
    accepted without a step, its residual ~9e-7 kg/s against shipped ~1e-12;
    ~1.5e-7 kg/s more enters the coil than leaves, which times the stream's
    enthalpy is the 2.4e-9 miss. `M69_FORCE=1` (`probe_predict.patch`, now with
    it): every solve of a predicted tick takes one Newton step from a converged
    start — the miss back to ~1e-15, residuals ~1e-12. `M69_FORCE=2`, the first
    solve only: the miss stays, the loose finish is the last re-solve's.

13. **The step costs 0–2 points** (`predict_mode2_forced.txt`, `counts3.py
    shipped predict_mode0.txt predict predict_mode2.txt forced
    predict_mode2_forced.txt`): the line flash's questions per tick against
    shipped — boiling 54 % held, 52 % after the step; stripped 65 → 66 % and
    39 → 41 %; cold 44 %.

14. **Stepping every predicted tick moved two resting plants** (`pump_gas_lock`,
    `water_slug` on Newton): their prediction equals their state, so the probe
    left them byte-identical, but the step polished each tick's rounding. A
    tick where no value moved is now not predicted (`M69_MOVED=0` turns that
    guard off): `pump_gas_lock` byte-identical again; `water_slug` still moves,
    by the step on its re-solves (12 numbers in 6 000 ticks, 1.4e-12 at most) —
    asked and accepted. The built engine's corpus: `corpus_*_built.txt`.

## To re-apply the probe

`probe_predict.patch` (its final state, with `M69_FORCE` and `M69_MOVED`) is on
58ffdc1: `git worktree add <dir> 58ffdc1`, then `git -C <dir> apply
probe_predict.patch`. Its four new files (`crates/core/src/probe.rs`,
`crates/solvers/src/probe.rs`, `tools/steptime/src/bin/m68_0_count.rs`,
`m69_0_count.rs`) are in it. The scripts here name the worktree
`W:/temp/claude/refinery-m68-0` and the output folder
`W:/temp/claude/m68-0-out`; both were removed at M69's close.
