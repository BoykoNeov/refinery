# Plan: refusals a player can read — DONE 2026-11-14

Ledger row F6 (`docs/DEFERRED.md`), widened from one message to all of them.

## The problem

`Engine::apply` refuses a bad command with a message, and the screens show
that message to the player as it is. 53 of those messages
(`grep -n '{node:?}\|{edge:?}\|{trip_id:?}\|{loop_id:?}' crates/core/src/engine.rs`)
print an id in its debug form:

    NodeId(21) ('vent_valve') is held at opening 1 by trip 'bed2_high', which
    is latched. Reset the trip first (`reset_trip`) … the valve can then be
    reopened by hand

Three things are wrong for a player: `NodeId(21)`; the command's code name
in backticks; and "reopened" when the move asked for was a close (F6).

## The change

Messages only. No behaviour, no codes, no numbers: **every plant stays
byte-identical** (a message is not in a snapshot — but run the corpus
anyway, both solvers, because the files are `crates/*/src`).

- [x] **Name, not id.** Wherever the node's name is already in the message
      (`{node:?} ('{name}')`), drop the id: `'{name}'`. Where only the id is
      printed (`"{node:?} is not a valve"`), look the name up
      (`self.graph.node(node).name`) and print `'{name}' is not a valve`.
      Same for edges and trips. **Exception — keep the id** in the two
      messages for an id that names nothing (`"{node:?} names no node"`,
      `"{edge:?} names no pipe"`, and the trip/loop equivalents): there is
      no name to print, and the id is what the caller got wrong.
- [x] **F6 itself.** In the trip-held valve refusal (near
      `is held at opening`), say what was asked: "… the valve can then be
      moved by hand" instead of "reopened". Do the same check on the pump
      and furnace versions beside it ("restarted", "relit" are right only
      if the refused command was a start; if the code cannot tell, use a
      neutral word).
- [x] **Leave the command names** (`reset_trip` and the like) — they are in
      backticks, a frontend can map them to its own key, and removing them
      loses information. This is a judgement call already made; do not
      reopen it.
- [x] **Tests.** `grep -rn "NodeId(\|EdgeId(\|TripId(" crates/*/tests` finds
      the tests that pin today's wording; update each to the new wording,
      nothing else. Then add ONE new test in
      `crates/godot-ext/tests/bridge.rs`: for every command variant sent to
      a node of the wrong kind on `scenarios/hydrocracker_hydrogen_quench.toml`
      (it has valves, pumps, a furnace, coolers, beds, loops and trips), the
      refusal's message contains no `Id(`. The storm test's `command`
      function (`crates/godot-ext/tests/command_storm.rs`) builds one of
      each variant; reuse its shapes.
- [x] The screens match on error **codes**, not text — confirm with
      `grep -n '"message"\|\.message\|begins_with\|contains' demo/*.gd` and
      fix any that match on words this changes.
- [x] Strike F6 in `docs/DEFERRED.md` with the commit's name, the house way
      (`~~F6~~ **CLOSED by …**`, the old text kept struck through).

## Done (2026-11-14)

- Every `{node:?}`/`{edge:?}` beside a name dropped; the id-only ones
  (`is not a valve`, relief and check valve, pump, furnace, cooler, water
  cooler, the "unreachable" source arms) look the name up. The four "names
  no …" refusals keep their id, as planned.
- F6: the valve reads "moved by hand". The pump's "started" and the
  furnace's "fired" were already right — their guard refuses only a start
  and a non-zero duty.
- Three strings this rewrote had broken whitespace (a `\n` plus indent in
  the boil-off vent refusal, runs of spaces in two condenser-cooling ones):
  restored to `\` line continuations.
- Tests: one pinned `"'furnace') is not a column"`
  (`reboiler_command_reference.rs`). The new test sweeps four plants — the
  hydrocracker, plus `crude_column`, `crude_unit_loops` and
  `crude_unit_overfill` for the column refusals — in 0.05 s.
- Screens: none matches on message text; they only print it.
- Corpus: 72 identical on both solvers, 0 moved.

**Left, outside this plan** (no id, but still not plain): the MANUAL→AUTO
"no measurement to transfer against" refusal has runs of spaces
mid-sentence (`SetControllerMode` in `engine.rs`); "measures Flow and this
setpoint is Level" and "would change its phase (Ok(Gas) to Ok(Liquid))"
print Rust debug forms; four tick-failure messages in the boil-off pass
have the same space runs.

## Gates

FULL tier, clippy, fmt, corpus on both solvers against the rolling baseline
(71 identical, 0 moved). If anything moved, a message leaked into a
snapshot somewhere: stop and report which field.

## Not in this plan

Refusals from loading a plant file (`crates/scenarios/src/validate.rs`):
their reader is the plant's author, and they are good as they are.
Tick-failure messages (`SimError::Numerical` and friends): they need
rethinking with `failed-tick-recovery.md`, once a player can actually act
on one.
