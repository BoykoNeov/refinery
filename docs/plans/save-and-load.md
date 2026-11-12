# Plan: save and load

Do `failed-tick-recovery.md` slice 1 first: this plan reuses its command log.

## The problem

Nothing survives closing the window. The crude unit started from empty takes
24 minutes of plant time to settle (`scenarios/crude_unit_empty.toml`); a
player who quits loses it.

## Slice 1 — a save is the plant file plus the commands (no engine change)

The engine is deterministic, so *the scenario source and the log of accepted
commands with their ticks* **is** the state. A save file is that, as JSON:

```json
{ "format": 1,
  "engine": "<workspace version from Cargo.toml>",
  "scenario": "res://scenarios/crude_unit_holdup.toml",
  "scenario_source": "<the TOML text as loaded>",
  "tick": 14400,
  "commands": [[1200, "{\"cmd\":…}"], [3900, "{…}"]] }
```

Keep the source text in the file, not only its path: a plant file edited
after the save must not change what the save means.

- [ ] `Session::save_json() -> String` and
      `Session::load_save(json: &str) -> String` in
      `crates/godot-ext/src/bridge.rs`. Loading builds the engine from
      `scenario_source` and replays to `tick` exactly as `rewind` does.
      Refusals, each with its own code: not JSON / wrong shape
      (`bad_save`), `format` not 1 (`save_format`), a replayed command the
      engine now refuses or a tick that now fails (`save_diverged` — this is
      what an engine change between saving and loading looks like; say so in
      the message and name the tick).
- [ ] Tests in `crates/godot-ext/tests/bridge.rs`, written first:
  1. round trip: run with commands, `save_json`, load it into a fresh
     session, the two `snapshot_json` strings are identical, and both stay
     identical for 500 more ticks with one more command sent to both.
  2. a save made **after** a `rewind` holds only the surviving commands.
  3. each refusal above, by its code.
  4. a save with the scenario's source altered by one number loads as the
     altered plant (the file is the truth, not the path).
- [ ] `binding.rs`: `save_json`, `load_save`. The scene writes the string to
      `user://saves/<name>.json` with `FileAccess` (only Godot can write
      `user://`; the bridge stays free of file I/O, as it is for loading).
      Godot-feature clippy.
- [ ] `demo/map.gd`: F5 saves to `user://saves/quick.json`, F9 loads it and
      shows "loading… replaying N ticks". The menu gets one row, "Continue
      (quick save)", shown only when the file exists.

**ASK the user** only this: where saves live (`user://`, the Godot default
under the user's profile — recommended — or a folder beside the project).

**Cost to state:** loading replays every tick. Measure and write down the
time for a 10-minute and a 60-minute save of `crude_unit_holdup`. If the
hour takes more than about a minute, that is the trigger for slice 2.

## Slice 2 — a save that holds the state itself (ASK FIRST)

Same fork as `failed-tick-recovery.md` slice 2, option B: the engine's state
serialized, so loading is instant and survives a long session. It also makes
a save tied to one engine version unless a migration is written. Do not
start it without the user choosing it there.

## Done when

Quit, start, "Continue", and the plant is where it was — proved by a test
that compares snapshots byte for byte.
