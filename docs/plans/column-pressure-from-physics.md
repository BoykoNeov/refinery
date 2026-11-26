# A column's pressure from physics, not a pinned number

Asked by the user on 2026-11-23: *"our aim is to make as real as reasonable
simulation, with toggleable elements (if the simulation gets too heavy, some
system can be switched to a simpler representation) — simulate 1.8 bar from
physics, not as a given number."*

This file follows `docs/plans/README.md`. It says what to read first, what each
slice changes, how to know it worked, and where to stop and ask. None of it is
a milestone until the user opens one.

## 0. What "from physics" means here (read this before anything else)

In a real crude unit, nothing in nature picks 1.8 bar. The designer picks it,
and the operator types it into the pressure controller on the reflux drum.
**So the number stays in the plant file, but as the controller's setpoint
(its target).** What changes is what the pressure *is*:

- **Today (most plants):** the column sits at exactly 1.8 bar. The engine pins
  it there, as if the controller were perfect and could never run out of reach.
- **From physics:** the pressure is a *state*, the result of a balance in the
  overhead:
  - in: the vapour the column sends up;
  - out: what the condenser turns back to liquid, plus any gas the vent lets out.

  The controller moves real equipment to hold the setpoint. While that
  equipment keeps up, the pressure sits at 1.8 bar. When it can't, the
  pressure moves the way a real column's would:
  - above the setpoint when cooling water is lost, the condenser floods or the
    vent is shut;
  - below it when the cooling water is cold, the feed is cut or the vent opens
    too far.

What is already built (M75, `docs/DESIGN.md` §80.1–§80.2) is half of this. A
column that holds liquid and declares `condenser_limit` already has its
pressure solved inside each tick, and that pressure rises above the setpoint
when the condenser can't condense what arrives. Only one plant uses it today:
`scenarios/crude_unit_overfill.toml`. What M75 does **not** do:

- the pressure never falls below the setpoint (the "flooded condenser" always
  copes on the low side, for free);
- there is no controller with tuning, a manual mode or a range to run out of;
  the hold is perfect until the condenser has no free area left;
- the cooling water is a fixed temperature, `coolant_c`, and its flow is not
  modelled;
- there is no gas in the overhead, so there is no vent and no vent-valve
  control (the most common scheme on a real crude unit).

## 1. Read first

1. `CLAUDE.md` (it overrides this file), then `docs/plans/README.md`'s rules.
2. `docs/DESIGN.md` §80.0–§80.2 and the M75.2/M75.3 parts after them
   (`grep -n '^## 80' docs/DESIGN.md`, then read from there). This is the
   condenser, the pressure solved within the tick, the drain and relief.
3. `docs/MILESTONES.md`, the M75 box.
4. `crates/core/src/graph.rs`: `HoldupState::column_pressure`,
   `condenser_ua` and `running_column_pressure`.
5. `scenarios/crude_unit_overfill.toml`: the one plant that declares the
   condenser.
6. The books (local, git-ignored, in `articles/`):
   - Kister, *Distillation Operation* (1990), ch. 17, §17.2: the pressure
     control schemes (vent / inert bleed, flooded condenser, hot-vapour
     bypass, coolant throttling) and how each fails. §17.2.2, pp. 528–531, is
     the flooded condenser M75 cites. The PDF has no text layer, so read the
     pages as images.
   - Rose, *Distillation Design in Practice* (1985), ch. 8, "Pressure
     Control". A vent valve on the condenser holds the pressure by how much
     inert gas it keeps; the gas blankets part of the condenser, and that sets
     how much condenses. Refinery feeds carry enough light gas (H₂ and light
     ends) that no extra gas needs to be fed in.
   - Ludwig vol. 2, ch. 8, and Kister ch. 9 for the relief side (already
     used by M75.3).

## 2. Lessons M75 paid for: do not relearn them

- **Solve the pressure implicitly, inside the tick** (backward Euler). The
  first M75 build lagged the column's answer to its own pressure by a tick
  and rang tick to tick with a gain of about 10 (§80.2). The column's
  boiling responds to pressure about ten times harder than the overhead's
  storage does.
- **Grow the search range from where the pressure stands**, never from a far
  end. A trial step a whole range away (3.52 → 1.8 bar in one tick) flashed
  the column and the tick was refused (§80.2, "the bracket grown…").
- **The network pins the column at its running pressure, one tick behind**
  (`running_column_pressure`), as it does every pin. Keep that. Do not make
  the column a free unknown in the network solve.
- **A column at rest must be bit-identical** to the same plant without the
  new key: the first trial at last tick's pressure, and the answer in one
  step when the equipment copes.

## 3. The ladder, and how a level is switched

Each level is a more physical representation of the same thing, so a heavy
plant can be dropped a level when it runs too slowly. How a level is chosen
is **fork A** (§5).

| level | what holds the pressure | cost per tick | new physics |
|---|---|---|---|
| 0, held | nothing; the column is pinned at its setpoint | none | none (today) |
| 1a, condenser limit | M75: pinned until the condenser can't cope, then rises | 1 step at rest, 2–few off the setpoint | built |
| 1b, controlled condenser | a pressure controller floods more or less of the condenser; the pressure can move either way | the same as 1a | controller, low side, cooling water |
| 2, vented | gas in the overhead, a pressure-control valve venting it | more (gas state, a vapour line) | light ends in the column, a vent line |
| 3, the overhead's own drops | drum pressure + condenser drop + line drop = the top's pressure | small | a few pipe drops |

The pressure drops of the trays (M73) already carry the top's pressure down
to the stages below. Every level reuses them unchanged.

## 4. The slices

### Level 1b: a controlled flooded condenser (no new components)

**1b.1 The controller.** The flooded share of the condenser (`1 − φ` in
§80.1) becomes the output of a PI pressure controller instead of "whatever
holds the setpoint exactly".
- Its measurement is the top's pressure; its setpoint is the file's
  `pressure_bar`. Its output is the flooded share, between a least and a most
  flooded value (game values in the file, with a cited default).
- Use the existing controller machinery (M8 `docs/DESIGN.md` §10, and M10 §12
  for pressure). A new controller kind is a fork, not a default.
- The pressure is still solved implicitly within the tick, with the
  controller's output fixed for the solve (a controller acting a tick behind
  is what a real one does; the pressure itself must not lag).
- Output at its "nothing flooded" end and still short: the pressure rises.
  This is M75's behaviour, reached through the controller now.
- Output at its "most flooded" end and still condensing too much: the
  pressure falls below the setpoint. **Fork B.**
- Manual mode and a held output by command, as other loops have, so a screen
  can take it by hand (M80's pattern, §83).

**1b.2 The cooling water.** The condenser's capacity depends on the water's
inlet temperature and flow, not a fixed `coolant_c`.
- Reuse the water cooler's condensing-side rating (`energy.rs`, the zone
  analysis, §67.13) rather than a new formula. The condenser's `UA` is still
  derived on the first tick unless the file declares it.
- Commands: the water's inlet temperature (summer/winter) and its flow (a
  cooling-water pump trip).
- What a player sees: cold water and the controller floods more; lose the
  water and the pressure climbs to relief.

**1b.3 The plant.** A NEW file, e.g. `scenarios/crude_unit_pressure.toml`,
copied from `crude_unit_overfill.toml` with the controller and the cooling
water declared. Stories, each measured on both solvers:
- cooling water lost, then restored;
- winter water (cold);
- the feed cut by half;
- the controller put in manual and the output frozen.

Then a key on the column screen and its measured table, as M75.4 did.

### Level 3: the overhead's own pressure drops (small, any time after 1b)

The setpoint is read at the reflux drum, as on a real plant. The column's top
then sits above it by the condenser's drop and the overhead line's drop
(pipe friction on the top vapour, already in `elements.rs`). Small, and
honest: on a real unit the top runs about 0.1–0.3 bar above the drum.

### Level 2: vented control with light ends (probe first)

The scheme most real crude units use: the crude carries light gas (methane
to butane, some H₂S). It comes up with the top vapour and does not condense
at the drum. A pressure-control valve vents it to fuel gas or the flare; how
much stays behind sets how much of the condenser it blankets, and so the
pressure (Rose 1985 ch. 8).

What makes this large:
- Today every node and pipe holds one phase, and a column takes liquids only
  (`validate.rs`, the gas-component refusals). The slate already allows gas
  components; the column does not.
- `validate.rs` refuses a partial condenser on purpose. Letting distillate
  leave as vapour breaks the energy argument M7 rests on (`docs/DESIGN.md`
  §5, fork 0: every kilogram that boils inside the column condenses inside
  it). **Fork C.**
- A vent line is a gas line from the drum, with a control valve, to a sink.
  `column_psv → flare` (M75.3) is the template: the column already has a
  vapour outlet.

So Level 2 starts with **2.0, a probe** in `tools/steptime/prototypes/`
(no engine change). It should answer: how much light gas a typical crude
carries (cite a source); whether the gas can be carried through the column
as an extra, never-condensing component without touching the energy datum;
and what the drum's gas inventory and the vent valve's flow look like on the
overfill plant's numbers. Put the answers to the user before any slice.

## 5. Forks: ask the user, in plain words, then wait

**ANSWERED by the user, 2026-11-23:** A (i), a `[fidelity]` key with a CLI
override; B yes, the pressure may fall below the setpoint; C as recommended,
Level 1b first, then the probe 2.0, then decide (leaning to the gas-only
vent); D new plant files only ("no point implementing something where it
won't be used"). The text below is kept as it was put.

**A. How a level is switched.** Two ways:
- (i) A key under `[fidelity]`, e.g. `column_pressure = "held" | "limit" |
  "controlled"`, backed by a trait in `core` with its implementations in
  `solvers` (CLAUDE.md rule 2), plus a CLI override like `--solver` so a
  heavy plant can be run a level down without editing its file. The cost: a
  file's condenser keys are then present but unread at "held", so the
  "declared only if used" rule needs an exception, reported in the snapshot.
- (ii) Per column, by what it declares (as M75 does today): simpler, but
  switching a level means a different plant file.
- **Recommend (i)**: it is what the user asked for ("switched to a simpler
  representation").

**B. Below the setpoint.** Letting the pressure fall below the setpoint
changes M75's "never below the set" rule. Only plants that declare the new
controller would see it, so `crude_unit_overfill` stays as it is.
**Recommend yes**: a column that can only err upward teaches half the story.

**C. Light ends (Level 2).** Either carry the light gas through the column
as a component that never condenses and vent only that gas (the M7 energy
argument survives), or lift the partial-condenser refusal (distillate leaves
as vapour; much larger, reopens M7 fork 0). **Recommend doing Level 1b
first, then the probe 2.0, then decide**, leaning to the gas-only vent.

**D. Which plants.** Levels 1 and up need a column that holds liquid
(holdup, M74). The six crude-unit plants without trays (boiling, cold,
steam, stripped, wet, wetter) cannot have holdup today (C22: no trays, free
water or steam), so they stay at Level 0. **Recommend: new plant files only**,
which CLAUDE.md requires anyway.

## 6. Gates for every slice

- Tests first (README rule 4): the failing test, then the change.
- Every existing plant byte-identical on both solvers against the rolling
  baseline (CLAUDE.md, "Which tests run when"), unless the slice names a move.
- Never edit an existing scenario file; ship a new one or an inline fixture.
- **A reference check per slice**:
  - 1b.1: the distillate's bubble point at the setpoint against the cooling
    water's temperature plus the condenser's approach. At 1.8 bar a crude
    unit's top is about 110–130 °C with 35 °C water. A setpoint the water
    can't reach must be refused or reported, not silently missed.
  - 1b.2: the condenser's duty against a hand calculation on its water side
    (`Q = ṁ_w · cp_w · ΔT_w`).
  - 2: the vent's gas flow against the light ends the feed brings in, at
    steady state.
- The books close every tick: mass to 1e-8 kg/s, heat to the step's
  tolerance (M75.1's standard).
- Cost measured with `tools/steptime`: no more than M75's few steps a tick
  while off the setpoint.
- Units newtypes, no panics, determinism (no new maps iterated).

## Checklist

- [x] Forks A–D answered by the user (2026-11-23, §5)
- [x] 1b.0 the switch: `[fidelity] column_pressure`, the trait, the CLI
  override — a pure refactor, every plant byte-identical (M89.0; fork A
  carried in no slice above)
- [x] 1b.1 the controller (M89.1, 2026-11-26; the share over the whole [0, 1], Kister giving no figures)
- [ ] 1b.2 the cooling water
- [ ] 1b.3 the plant, the screen key, the measured table
- [ ] 3 the overhead's own drops
- [ ] 2.0 the light-ends probe, put to the user
- [ ] 2.x vented control (sliced after 2.0)
