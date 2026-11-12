# From simulator to game — where the project stands and what it lacks

Written 2026-11-12, after a review of the whole project asked for as "improve
the project: design and concepts, visuals current and future, fix bugs, write
plans for what is too big". This file is the concept half of that review. It
is a statement of direction, not a milestone: nothing here is built until a
plan under `docs/plans/` is taken, and each plan's forks are still the user's
to decide.

`DESIGN.md` argues the physics, `ROADMAP.md` records the milestones,
`DEFERRED.md` lists the open physics. None of them asks the question this
file asks: **what does a person who is not the author do with this?**

## 1. What exists (measured, 2026-11-12)

- **An engine that does not fall over.** 71 plants, two solvers. Run ten times
  longer than the usual check (60 000 ticks instead of 6 000), every plant
  finishes, on both solvers. Sent random commands — valid ones, absurd
  ones, commands for the wrong kind of equipment — on all 71 plants, the
  engine never crashed and never reported a number that was not a number. It
  refuses, or it fails a tick with a reason. This is now a permanent test
  (`crates/godot-ext/tests/command_storm.rs`).
- **Deep physics in a narrow place.** The crude column has had fifteen
  milestones (M60–M75): heat balance, steam stripping, trays that flood,
  liquid holdup, start from empty. The reactor, the furnace and the pump have
  had a few each. Tanks, valves, relief and control loops are solid.
- **Five hand-drawn screens** for 23 of the 71 plants, each a story with its
  own keys, each drawn by its own script (7 000 lines of GDScript, most of it
  repeated between them).
- **As of this review**: a menu that lists every screen and plant
  (`demo/menu.tscn`, now the project's opening scene), screens that scale to
  the window, and a **plant map** (`demo/map.tscn`) that draws any of the 71
  plants from its own data — watching only.

## 2. What a game needs that is missing

In the order a player would hit them. Each has a plan; the plan's file is
named.

### 2.1 A failed tick ends the session — `plans/failed-tick-recovery.md`

The engine's rule is "a solver that cannot settle returns an error, never a
wrong number". That is the right rule for the engine and the wrong place to
stop for a game: **today the error is terminal**. Measured on 108 failed
ticks: not one plant ticked again afterwards, and in 15 the failed attempts
still changed what the screen shows. So every screen prints `HALTED` and the
player restarts from the beginning.

The project's answer so far has been to keep the player away from the edge:
the column screen's keys stop at a measured safe region (M65–M72 spent much
of their time measuring those regions). That works for five keys on one
screen. It cannot work for a player with a whole plant under their hands.

What is needed is the thing every simulation game has: **the last good
moment, kept, and a way back to it** — "that last move broke the plant;
here it is as it was two seconds ago, and here is what the engine said".
The engine is deterministic, so the first version needs no engine change at
all: replay.

### 2.2 No save, no load — `plans/save-and-load.md`

Nothing can be kept between sessions. A cold start of the crude unit takes
24 minutes of plant time; a player who quits loses it. Same mechanism as 2.1
(the plant's file plus the commands sent, replayed), then a real stored
state when replay gets too slow.

### 2.3 Only 23 plants can be seen — `plans/plant-map-screen.md`

Each screen is hand-drawn for its plants. The other 48 plants run only in
tests and the command line. The plant map added in this review is the first
cut of the alternative: one screen that draws whatever it is given. What it
still lacks is the point of a game — hands. No valve can be clicked, no pump
stopped, no pipe punctured.

### 2.4 Nothing asks anything of the player — `plans/objectives-and-alarms.md`

Each screen is a story the author already knows. There is no goal ("make
40 kg/s of on-spec diesel"), no cost (fuel burned, product lost to slop,
equipment broken), no alarm list (the trips and limits exist, but each
screen draws them its own way and none of them collects "what is wrong right
now" in one place). A plant that is running fine and one that is quietly
flooding its trays look alike until a number turns red.

This is the largest design question in the project and it is **the user's**:
is this a sandbox (learn the plant by breaking it), an operator's game (keep
it running through upsets), or a builder's game (lay out the plant)? The plan
lists what each needs from the engine; it does not choose.

### 2.5 The engine's words are an engineer's — `plans/refusal-messages.md`

53 refusals name equipment as `NodeId(21) ('vent_valve')`. A player reads
them on screen. Ledger row F6 is one of them; the plan takes all of them.

### 2.6 The screens repeat each other — `plans/screen-kit.md`

Five scripts each carry their own colours, their own text helper, their own
load-tick-shoot loop, their own key legend. The newest plants outgrew the
column screen's layout (text over text, lines off the window's edge — fixed
in this review). A shared kit makes the next screen a day's work instead of
a thousand lines, and gives every screen the same look by construction.

## 3. Visual direction

What the screens already get right, and should keep:

- **Every number on screen is the engine's.** No screen computes physics.
  Keep it a hard rule; it is why the screens can be trusted.
- **Dark page, one accent per meaning**: blue cold, red hot, orange fire,
  yellow warning, red alarm, green good. Already consistent across screens by
  copy-paste; the kit makes it consistent by construction.
- **The story in the title.** "A hydrogen bed whose runaway breaks the wall
  twice over" tells a player what to look for.

Where they should go, cheapest first:

1. **One page, three zones, on every screen.** The plant in the middle, the
   readings on the right, what just happened and the keys along the bottom.
   The screens are close to this already; the kit should make it the frame
   rather than a habit.
2. **Motion where there is flow.** A pipe's flow is a static width today.
   Dashes moving along it at a speed set by the flow make "is it flowing,
   which way" readable without a number. One shared pipe-drawing function.
3. **An alarm strip.** One line at the top of every screen: what is past a
   limit right now, newest first, in the engine's own words. Empty and dim
   when all is well. (Needs 2.4's collected alarms.)
4. **Trends.** The reactor screen has a 30-minute chart; nothing else does.
   A column whose products take minutes to answer a key (M74) is unreadable
   without one. A shared chart, any snapshot number against time.
5. **The map as the home screen.** The player opens a plant on the map,
   sees it whole, and opens a unit (the column, the reactor) into its
   hand-drawn screen — the same running plant, not a different file. This
   is what turns five demos into one game. It needs one simulator shared
   between scenes instead of one per scene.
6. **Drawn equipment.** Shapes are rectangles and circles drawn in code.
   That is honest and scales; proper symbols (the standard process-diagram
   shapes for a pump, an exchanger, a column) would read better and are a
   drawing job, not an engine job. Last, because it changes nothing a
   player can do.

## 4. What should NOT change

- The engine/frontend boundary: commands in, snapshots out, JSON across.
- Determinism. It is what makes replay — and so recovery and saving — nearly
  free. Any plan that would weaken it should be refused.
- The habit of probing before building and of the user deciding the forks.
  The plans below are written for a less capable model to carry out, and
  each marks where it must stop and ask.

## 5. A note on where the effort has gone

Fifteen consecutive milestones on one column. Each was asked for and each is
good work; the column is now far ahead of everything around it, and far
ahead of the game that would show it. A plant with a superb column, no save
file and a session that ends at the first failed tick is a simulator
demonstration. If the aim is a game, the next milestones after M75 are better
spent on section 2 than on the column's next refinement. That is a
recommendation, not a decision.
