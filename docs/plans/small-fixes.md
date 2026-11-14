# Plan: small fixes found in the 2026-11-12 review

Each is independent. One commit each. None changes a plant's numbers.

## 1. The corpus check does not notice a plant that vanished — DONE 2026-11-12

`crates/cli/src/main.rs`, `fn corpus`. With `--baseline`, each row of the
new run was graded against its namesake in the baseline; a plant in the
baseline and **missing from the run** (a deleted or renamed file) was never
looked at, so the run exited 0 having silently lost a plant.

- [x] `missing(rows, baseline)` and a failure naming the plants, when at
      least one whole directory was run (a run of a few named files is
      meant to leave the rest of the baseline out). Unit tests on `missing`
      and `verdict`; an end-to-end test in
      `crates/cli/tests/corpus_exit_status.rs`.
- [x] The vanished plants are rows of the printed table too, verdict
      `**missing**` (2026-11-14), listed only when whole directories were
      run, as the failure is; a run of named files lists none (tested).

## 2. `refinery run` writes unbuffered — DONE 2026-11-14

- [x] Buffered, flushed with its error reported; 1 000 ticks of
      `tank_pump_valve.toml` byte-identical before and after, to a file and
      to stdout.

Same file, `fn run`: `serde_json::to_writer` straight onto a `File` or
stdout makes one system call per token. Wrap the sink in
`std::io::BufWriter` and `flush()` before returning (propagate the error).
Check the output of
`refinery run scenarios/tank_pump_valve.toml --ticks 1000` is byte-identical
before and after (`cmp`).

## 3. The CI comment in the README drifts

`README.md`'s status paragraph was three months stale when found (it said
"M9 is open" at M75). It now points at `CLAUDE.md` instead of naming a
milestone. Nothing to do unless someone re-adds a milestone number there;
if a review finds one, replace it with the pointer again.

## 4. A command's value has a floor but no ceiling — DECIDED 2026-11-14: (a), leave it

The user chose (a): no ceilings in the engine. A frontend limits its own
inputs, and `failed-tick-recovery.md` makes the failed tick survivable.


`Engine::apply` refuses a negative or non-finite duty, pressure,
temperature or flow, and accepts `1e300` W, `1e300` Pa or a puncture of
300 m² in a 0.1 m pipe. The tick after such a command fails (correctly, with
an `Err`), which today ends the session. Found by the command storm; not a
crash.

Whether to refuse these at the command is a design choice, so **ask the
user**, offering:

- (a) leave it: a frontend limits its own inputs, and
  `failed-tick-recovery.md` makes the failed tick survivable — recommended;
- (b) refuse a puncture wider than the pipe's own bore (the one case with
  an obvious physical ceiling), nothing else;
- (c) ceilings on every command, each a number someone must choose.

## 5. `Refinery.cmd` and a stale Godot cache

`Refinery.cmd` runs the editor once only when `.godot\extension_list.cfg`
is missing. A clone made before a new scene or script was added still has
the file, and Godot run without the editor does not re-import. Nothing
broke on 2026-11-12 (the new scripts use `preload`, not `class_name`), so
this is a note, not a bug: **never add a `class_name` to a `demo/` script**
without also making `Refinery.cmd` re-run the editor when `demo\` is newer
than the cache.
