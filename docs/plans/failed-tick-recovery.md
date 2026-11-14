# Plan: a failed tick is not the end of the session — slice 1 DONE 2026-11-14 (M76)

## The problem, measured

`Engine::tick` returns `Err` when the plant cannot be solved. Every screen
then prints `HALTED` and stops for good. Measured 2026-11-12 with random
commands on all 71 plants (the probe that became
`crates/godot-ext/tests/command_storm.rs`): 108 failed ticks.

- **0 of 108** ticked successfully when `tick()` was simply called again
  (five retries each). The cause of the failure is still in the plant.
- **15 of 108** changed the snapshot during those failed retries. `tick`
  runs the trips, relief latches, burn-outs and control loops *before* the
  solve (`crates/core/src/engine.rs`, the numbered steps at the top of
  `pub fn tick`), so a failed tick is not all-or-nothing.

So a frontend cannot recover by retrying, and cannot trust the state it is
left holding.

## The idea

The engine is deterministic: the same plant file and the same commands at
the same ticks give the same state, to the bit (CLAUDE.md rule 3). So the
state at any past tick can be rebuilt from **the file and a log of the
commands that were accepted, each with the tick it was sent at**. No engine
change is needed for a first version.

## Slice 1 — rewind by replay, in the bridge (no engine change, no decision) — DONE 2026-11-14

**What changed from the text below while doing it** (DESIGN §81):

- **`recover` goes back to just before the last accepted command**, however
  many good ticks ran since — the user's choice (2026-11-14) when asked about
  a command whose harm shows ticks later (1 GW into the knockout drum fails
  four ticks on), which "back one tick, undo what was sent since" leaves in
  place to fail again. Called again it undoes the one before; with none left,
  `nothing_to_undo`.
- **A refused command IS recorded and replayed.** `Engine::apply` notes that a
  command arrived before judging it, which changes the next tick on a vapour
  plant; leaving it out rebuilt a different plant (test 4, seen to fail).
  Only what the bridge refuses (bad JSON, unknown id) is left out.
- **A failed tick IS recorded and replayed** (found at review): the attempt
  moves the plant before it fails, so a frontend that carries on after one
  has a plant a replay must take through it too.
- The record and the replay live in `Bridge` (Rust `Result` API);
  `Session` wraps them as JSON. New codes `bad_rewind`, `nothing_to_undo`,
  `replay_diverged`.
- **Screens: the pump screen, not the map.** The map sends no commands yet,
  so it has nothing to undo; `plant-map-screen.md` slice 2 wires it. The pump
  screen's gas-lock plant fails after 73 presses of D (3.15 K); `--break-at=`
  on an `--auto` run sends that value to see it. No "rewinding…" line: a
  rewind measured 0.7 ms a tick on the slowest plant.


All in `crates/godot-ext/src/bridge.rs` (pure Rust, tested by `cargo test`)
plus one forwarding line each in `binding.rs`.

- [x] `Session` keeps: the scenario source text it loaded, and
      `Vec<(u64 /*tick index when applied*/, String /*command JSON*/)>` of
      every command the engine **accepted** (a refused command is not
      logged). Cleared on load.
- [x] `Session::rewind(to_tick: i64) -> String` (JSON `null` or
      `{code,message}` like every other call): rebuild the engine from the
      kept source; tick it to `to_tick`, applying each logged command when
      the tick index equals the tick it was logged at; drop log entries
      logged at `to_tick` or later. Refuse `to_tick` greater than the
      current tick index or negative with a new code `bad_rewind`.
- [x] `Session::last_good_tick() -> i64`: the tick index after the last
      successful tick (it is just the engine's tick index while nothing has
      failed; keep it in a field so it survives a failed tick).
- [x] `Session::recover() -> String`: `rewind(last_good_tick)` minus the
      commands logged since the tick before it, i.e. rewind to the last good
      tick and **drop the commands sent after the last successful tick**
      (they are what broke it). Returns JSON
      `{"rewound_to": N, "dropped": ["<command json>", …]}` so the screen can
      say what it undid.
- [x] Tests in `crates/godot-ext/tests/bridge.rs` (same style as its
      neighbours), each written first and seen to fail:
  1. replay is exact: run plant A 300 ticks with three commands along the
     way, take `snapshot_json`; `rewind(300)` on the same session gives the
     identical string; so does a fresh session fed the same log.
  2. `rewind(150)` then ticking to 300 **without** the later commands gives
     the snapshot of a run that never sent them.
  3. recovery: a plant, one accepted command, and a tick that then fails.
     The storm saw `scenarios/tank_temperature_control.toml` fail with
     "cools below absolute zero" some ticks after `set_source_temperature`
     of `1e-12` on node 0, and `scenarios/vessel_pressure_control.toml` fail
     on the next tick after `set_reservoir_pressure` of `1e300` on node 3 —
     but each came after other commands, so FIRST confirm one fails when
     sent alone (the second should); if neither does, print a failing
     sequence from the storm test's generator and use that. Then: tick until
     it fails; `recover()`; the session ticks again for 500 ticks, and the
     dropped list holds exactly the commands sent after the last good tick.
  4. a refused command is not logged (send a valve command to a tank;
     `rewind` after it replays identically).
  5. `rewind` before `load` → `not_loaded`.
- [x] Mutation pass per CLAUDE.md on the new bridge code (targeted:
      `cargo test -p refinery-godot-ext --test suite bridge::`).
- [x] `binding.rs`: `rewind`, `recover`, `last_good_tick` — one line each.
      Run the godot-feature clippy (CLAUDE.md) and record it.
- [x] (the pump screen instead — see above) One screen first: `demo/map.gd`. On a failed tick, instead of
      `_halt`, call `sim.recover()`, show
      "the plant could not be solved — put back to t = … ; undone: …" with
      the engine's message under it, and carry on. Then the pump screen.
      Leave the column screen's measured regions alone.

**Cost to state in the commit:** replay takes as long as the ticks took.
Measure it on the slowest plant (`crude_unit_holdup`, ~5–8 ms a tick): a
rewind after 10 minutes of plant time is 6 000 ticks, 30–50 s. That is why
slice 2 exists. For slice 1, show "rewinding…" and do it.

**Do not**: change `Engine`; make `tick` transactional; catch the error and
keep ticking. Those are slice 2 or wrong.

## Slice 2 — checkpoints, so a rewind is short (ASK FIRST)

Replay from tick 0 does not scale to an hour's play. The fix is to keep the
engine's whole state every N ticks and replay only from the nearest one.

This needs `Engine` to be copyable, and it is not: it owns seven
`Box<dyn …>` models (`pub struct Engine` in `crates/core/src/engine.rs`) and
several of them carry memory between ticks (the line flash remembers a tick,
the flow solver its last answer, the column its profile). A copy that misses
one of those memories replays differently — silently.

**Stop and ask the user** between these, with a measurement in hand:

- **A. Clone the engine.** Add a `box_clone` to each solver trait, derive
  `Clone` down the tree, and prove it with a test that clones mid-run on
  every plant and requires both copies to stay byte-identical for 6 000
  ticks. Touches `core` traits (a hard-rule area) but is mechanical.
- **B. State as data.** Serialize the engine's state (serde) — which is
  also what a real save file needs (`save-and-load.md`, slice 2). More work,
  one mechanism for both.
- **C. Do nothing more**; accept slow rewinds, cap the log.

Recommendation to offer: A first (it is testable to the bit with the corpus
tool), B when saves need it.

## Done when

A player who breaks a plant on the map screen sees what happened, the plant
as it was a moment before, and can carry on — and `cargo test` proves the
rewound plant is the same plant.
