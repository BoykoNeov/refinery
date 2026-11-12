# Plan: something to do, and something that says what is wrong

**This plan starts with a question to the user and builds nothing until it
is answered.** It is here so the question is written down with what each
answer costs.

## The question

What is the player for?

| | the player's job | what a session is | needs from the engine |
|---|---|---|---|
| **A. Sandbox** | poke the plant, see what happens | open-ended | recovery and saves (`failed-tick-recovery.md`, `save-and-load.md`); nothing else |
| **B. Operator** | keep a given plant running and on target through upsets | a shift: a list of events, a score at the end | A, plus **alarms**, **targets**, **a score**, **scripted upsets** (below) |
| **C. Builder** | lay out a plant from parts | a build, then B on what was built | B, plus plants made at run time (`load_scenario_source` exists), a parts list, placement rules, costs — and far better error words from `validate.rs` |

The engine today suits **B** best: 71 plants are 71 situations, the trips,
relief valves, burst tubes and failed walls are upsets already, and every
screen's `--auto` timeline is a hand-written shift. A is B without the
scoring. C is a different, much larger project.

Recommend B, built so A falls out of it (a shift with no targets).

## If B: what to build, in order

Nothing here changes a plant's numbers; everything is new data out, or
frontend.

### 1. Alarms as one list (engine, small)

Today "what is wrong" is spread over the snapshot: trip states, a relief
valve's lift, a tank's overflow, a pump's `cavitating`, a column's
`drum_limit`/`base_limit`, flooded trays, a furnace's burst tubes, a bed's
failed wall, a loop at its limit. Each screen hunts for the ones it knows.

- [ ] `Snapshot::alarms: Vec<Alarm>` — `{ node | edge, kind, since_tick,
      severity: notice | warning | alarm }`, **derived from fields the
      snapshot already has**, in one function in `core/src/snapshot.rs`. No
      new physics, no thresholds invented: an alarm exists only where the
      snapshot already says a limit is passed.
- [ ] Written only when non-empty (`skip_serializing_if`), so a plant with
      nothing wrong keeps its fingerprint — M38's `by_hand` precedent
      (`docs/DEFERRED.md`, row F2). Plants that do raise one move, wire only: say which,
      and get the user's nod before refreshing the baseline.
- [ ] The map and the kit's alarm strip (`docs/GAME.md` §3.3) read it.

### 2. Targets (scenario file, new optional table)

- [ ] `[[targets]]` in a **new** plant file (never an existing one):
      a pipe's flow at least X, a product's cut within a range, a tank
      between two levels, a temperature under a limit. Read-only to the
      engine: it reports each as met or not in the snapshot. A probe
      first, on one plant (`crude_unit_holdup`), to find what a sensible
      target is — ask the user with the numbers.

### 3. Events (scenario file, new optional table)

- [ ] `[[events]]`: at tick N, apply this command (a feed's temperature
      drops, a pump trips, a pipe is punctured). The engine already takes
      every one of these as a `Command`; this is a scheduler in the bridge,
      and the screens' `TIMELINES` are its prototype. Deterministic by
      construction.

### 4. A score (frontend)

- [ ] Product made on target × time, minus fuel, minus what went to slop,
      flare or the ground, minus broken equipment. Every term is an
      integral of a snapshot field. The weights are a game-design choice:
      ask.

## What to ask the user, in plain words

1. Sandbox, operator, or builder? (Recommend operator.)
2. If operator: one plant done well first (which?), or a thin version
   across all five units?
3. Should an alarm ever stop the clock (pause on first alarm), or only
   show?

Then open a milestone for step 1 alone.
