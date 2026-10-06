# Milestone history

Every milestone's close-out report, moved here verbatim from `CLAUDE.md` on
2026-10-03 so the project guide stays under Claude Code's instruction-size
limit. Newest first (with the order the guide had, not strictly by date).
Since M35 (2026-10-03), `CLAUDE.md` keeps only a one-line pointer to the
latest milestone; each close-out box is written here, at the top of the list
below, when its milestone closes.

References elsewhere in the repo to "CLAUDE.md's M17 box" and the like (mostly
in `docs/DESIGN.md`) now point at this file.

---

**M48 is CLOSED (2026-10-06): valve memory — a relief that pops and reseats
below its set; `docs/DEFERRED.md` B6's gas clause closed, A22 re-measured.** On
the user's request ("work on valve memory") and DECISIONS: both readings of it,
hysteresis first; mid-slice, refuse a pop valve with no vessel behind it rather
than let it chatter; and, once the second reading was measured, close after the
first. DESIGN §53.
- **`blowdown_bar` on a `relief_valve`** (M48.0): shut, the valve runs the M5
  curve bit for bit; the tick after its inlet stands above set it goes to full
  lift and holds it until its inlet falls below `set − blowdown`. The latch
  (`Blowdown { amount, lifted }` on the node) moves at the top of the tick from
  the last solve's pressure, beside the trips, and rides in the snapshot on the
  node's `kind`. `FlowSolver` unchanged.
- **Refused at load** in liquid service, and in gas with no vessel reachable
  behind the inlet through zero-volume nodes (`require_blowdown_cushion`): with
  nothing to store pressure the latch flips every tick. Not promised: a vessel
  reached only through a shut operator valve, or behind an inlet line that
  loses more than the blowdown — the latter chatters, as API 520 Part II's
  inlet-loss rule warns, and is a gate.
- **Demo `relief_pop_cycle.toml`** (42nd file): 94 lifts in 6 000 ticks, the
  receiver a saw-tooth between 18.67 and 20.03 bar, lift and reseat on the same
  ticks on both fidelities.
- **M48.1 measured, not built**: nothing reaches the chatter refusal but a
  stub, and on A22 the game solver keeps its previous answer while Newton
  returns to zero flow whatever came before. First reported as "both already
  keep last tick's answer" (300 repeat solves, which cannot tell memory from
  preference) and corrected the same day; the user closed on the first
  reading and was told of the correction. Neither rule that settles A22 was
  taken.
- Gates: six in `relief_blowdown_reference.rs`. Mutations: six, all caught by
  the gates predicted, three by one more each as a consequence. Two gate
  readings corrected while building (chatter is a rate, not every tick; the
  hysteresis reads the tick's own pressure).
- 41 plants byte-identical on both fidelities, 1 new. Release property tests
  pass. The Godot binding did not change.

**M47 is CLOSED (2026-10-06): a dead end stands where its own head puts it —
`docs/DEFERRED.md` A22's CI failure closed, the row kept open.** On the user's
request ("start a22") and DECISION ("fix gravity error only", over Newton
refusing the case or a relief that shuts against back-pressure). DESIGN §52.
- **The fault was the head, not the pair**: the random chain arm already allows
  a relief in reverse flow two answers if both are exact roots. The game
  solver's was (1.2e-7 kg/s); Newton's was not (3.0e-5 at zero throughput),
  because the stood stretch read its 3.8 m drop's gas head from the pass it
  was parked in.
- **The tie re-stands the stretch from its own recompile** until no node moves
  more than 1e-6 Pa, at most eight times; gas settles in two, Newton's answer
  recompiles to 2.3e-12 kg/s. A column that will not settle is not a tie.
- **Both answers stay**: zero flow (Newton) and 0.2708 kg/s backwards (game
  solver). Which one a relief held shut by back-pressure takes is element
  state, B6 — A22 stays open for that.
- Gate: `a_dead_end_tie_in_gas_stands_on_an_exact_root` (was the A22 known
  defect; written failing first). Mutations: three, all caught; the zero-cap
  one by a different assertion than predicted (§51's re-run takes Newton to
  the other answer, not to a refusal).
- 41 plants byte-identical on both fidelities. Release property tests pass.
  The Godot binding did not change.

**M46 is CLOSED (2026-10-06): a repeat onto a classification that never
converged is re-run, not refused — `docs/DEFERRED.md` A20 struck.** On the
user's request ("retry the solve rather than refuse"). DESIGN §51.
- **Newton answers two reliefs in series**: its cold first pass fails, the
  second converges and points back at it, and the active-set loop called that
  chatter. Chatter is two answers; the repeated classification had never been
  one. Such a repeat is now re-run once from the converged pass's answer, and a
  re-run that fails returns the refusal it postponed. 68.354 kg/s, as the game
  solver. The dead-end tie is still asked first; a stretch below vacuum is
  refused outright (`Tie::BelowVacuum`).
- **General, not narrow**: built first as the ledger's "re-run from the stood
  pressures"; a mutation removing that seed passed every gate, so the rule
  dropped the dead-end shape. Covered by argument for stretches with two ways
  in or a vessel; the only fixture is the A20 chain.
- **A22 (new, older than M46)**: the mutation runs drew a gas chain, drive
  backwards through two reliefs, where §50's tie answers zero flow and the game
  solver 0.2708 kg/s backwards — both stand. Pinned as
  `known_defect_a_dead_end_tie_picks_one_of_two_answers_in_gas`, not fixed.
- Gates: `two_reliefs_in_series_answer_on_both_fidelities` (was the A20 known
  defect), `a_retry_that_fails_is_refused_as_the_cycle_it_postponed`. Mutations:
  five, four caught as named, one uncaught as predicted possible (re-running a
  below-vacuum stretch: the only fixture's re-run fails anyway).
- 41 plants byte-identical on both fidelities. Random arms unchanged. Release
  property tests pass. The Godot binding did not change.

**M45 is CLOSED (2026-10-06): a loop holds its valve while its pump is
stopped — `docs/DEFERRED.md` E25 struck.** The user's DECISION: E25 as the next
slice, told plainly no shipped plant reached it yet; "Controller holds" over a
trip that parks the valve; and twice, on faults the slice found before
building, "Fix it first" / "Fix the solver first". DESIGN §50.
- **A pump started against its shut valve runs** (M45.0): with a check valve
  between them, the stretch is a dead end — filled it shuts the disc, parked low
  it opens it — and the active-set loop refused the tick as chatter on both
  fidelities. The cycle branch now recognises a dead end's tie and stands the
  stretch at zero drive. M8.0's chatter stub was this tie and is now a relay; the
  relief arm's refusal floor became a ceiling. Newton's refusal of two reliefs in
  series (present before) is pinned: A20.
- **The game solver opens a cracked valve behind a check valve** (M45.1): every
  fill opening up to 0.3% diverged at 5 000 sweeps on the demo; the node's
  Newton step was ~400× too long and every step refused. A node the ladder
  cannot move is now solved by bisection on its bracket: 6–15 sweeps, on
  Newton's flow. Newton's own stall on the fixture at 1% is pinned: A21.
- **`on_pump_stop = { pump, output }`** (M45.2): in AUTO, with that pump off, the
  loop holds its valve and tracks it; the restart ramps (0.0043 kg/s on the
  first tick against 24.80). Refused on P loops, non-valves and cascades (E29).
  `ControlSnapshot::on_pump_stop`, skipped when absent.
- **Demo `tank_level_fill_pump_hold.toml`** (41st file): a self-resetting pump
  trip restarts the pump four times in 6 000 ticks, each a ramp; its twin
  without the key surges to 23.6 kg/s every restart and cycles seven times.
- Gates: `dead_end_disc.rs`, `cracked_valve_disc.rs`, `pump_stop_reference.rs`,
  gates 9 and 10 of `check_valve_reference.rs`, the relay stub and the A20/A21
  known defects. Mutations: 9 + 5 + 7, every one caught or uncaught as
  predicted but one: H3 escaped the gate named for it (gate 1 had no lower
  bound on the restart), and the bound was then added.
- 40 plants byte-identical on both fidelities after each slice, 1 new. Release
  property tests pass; random disc chains converge 392/400 (Newton) and 373/400
  (game) from 366 and 355. The Godot binding did not change; rebuilt with
  M45.1's game solver, its clippy clean, all five furnace `--auto` runs and the
  M6.2 demo replayed with every recorded event on its tick.

**M44 is CLOSED (2026-10-06): a restart asks the trips that do not hold the
equipment — `docs/DEFERRED.md` E28 struck (its last clause).** The user's
DECISION: E28's last clause as the next slice; told it was two problems, "Both
cases"; a restart refused for a trip on other equipment "Stays dark for a
person". DESIGN §49.
- **Another trip on the same equipment** (M44.0): a person's reset between
  ticks asked only its own trip, so a second trip on the furnace whose reading
  had just crossed its limit was still armed, and the reset relit the furnace
  for nothing: `Ok`, 3 MW on the snapshot, cut at the top of the next tick.
  Proved by a fixture first. `restart_bars` now asks every armed trip on the
  equipment against a fresh reading: `trip_about_to_fire`, and the furnace stays
  dark for a person. No plant moved.
- **Start permissives** (M44.1): `restart_permissives = ["<trip>", …]` on a
  trip whose reset restarts. Every trip named by a trip that held the equipment
  must be clear when the stop ends (armed, its fresh reading outside its
  condition), or it stays stopped for a person: `permissive_not_clear`. Five
  load-time refusals; `TripSnapshot::restart_permissives`, skipped when empty.
- **Demo `furnace_restart_permissive.toml`** (40th file), key 5 on the furnace
  screen (K starts the pump): the overfill trip stops the feed pump at 266, the
  starved heater's tube trip cuts it at 299 and re-arms itself at 321, and the
  heater stays dark, saying why. Without the line: 88 cuts and relights in 6 000
  ticks. A person resets the pump's trip, starts the pump and relights at 400.
- Eight gates, `tests/restart_permissive_reference.rs`, plus the screen's fifth
  timeline. Eight mutations, all caught.
- 39 plants byte-identical on both fidelities after each slice, 1 new. Release
  property tests 223/223; the Godot binding did not change; all five `--auto`
  runs replayed.

**M43 is CLOSED (2026-10-05): why a stop did not restart, in the snapshot —
`docs/DEFERRED.md` F4 (logged and struck in one go).** The user's DECISION, on a
review finding after M42 ("fix this, let the player have an indication or
message or information"): the engine kept the reason a furnace stays dark in a
private record, dropped the moment the trip let go. DESIGN §48.
- **The record keeps WHY** (M43.0): `HeldEquipment::restartable` becomes a set
  of `RestartBar`s; `restart_bars` adds M41's tubes-in-place check and is the one
  verdict the release acts on. Byte-identical on all 38 plants.
- **`NodeSnapshot::trip_stop`** (M43.1): `held` with `barred_by` while a trip
  holds the equipment; `not_restarted` with `at_tick` and `barred_by` after the
  last let go without handing it back, while the equipment stands where the
  trip left it; absent otherwise. Reasons: `reset_restarts_nothing`,
  `pressed_by_hand`, `tubes_burst_during_stop`, `tubes_burst` (only the last
  lifts with new tubes).
- **Demo `furnace_burst_during_stop.toml`** (39th file), key 4 on the furnace
  screen: tubes burst on the trip's tick 75, new tubes at 80, the trip re-arms
  itself at 128 and the heater stays dark, the panel, the furnace and the
  message line saying why; a person relights it at 200. The older stories now
  say "NOT RELIT … the emergency stop was pressed" after a reset.
- Six gates, `tests/trip_stop_reference.rs`, plus the screen's fourth timeline.
  Seven mutations, six caught; the new-stop clear is uncaught as predicted.
- 32 plants byte-identical, 6 moved wire only (every plant whose trip fires;
  `tank_level_fill_check_valve` was not predicted), 1 new; field cut out, all
  byte-identical. Release property tests 223/223; the Godot binding did not
  change.

**M42 is CLOSED (2026-10-05): a burst during a stop makes its restart a
person's — `docs/DEFERRED.md` E28's replaced-tubes clause struck (its last clause
open).** The user's DECISION, on the question M41 left in E28: a burst while a
trip holds the furnace dark makes that stop's restart a person's, even after new
tubes. DESIGN §47.
- **The burst marks the stop** (M42.0): the burn-out pass clears the trips'
  record of the stop for a restart (`HeldEquipment::restartable`, the flag a
  hand-pressed stop already clears) when it bursts a furnace a trip holds. The
  trip still re-arms; the furnace stays dark, its loop in MANUAL. New tubes do
  not undo it; a burst on a lit furnace marks nothing, so the next stop restarts
  as before. Three lines in `run_burnouts`; no interface changed.
- Four gates, `tests/burst_during_stop_reference.rs` (burst on the trip's tick
  75, new tubes at 80, dark past the self-reset at 128 where the intact twin
  relights; the same through a person's reset, then a person's AUTO fires the new
  tubes; a fire bursts the LIT tubes between two stops and the second still
  relights at 3 MW; a stop with no burst relights). Four mutations, all caught
  by the gates predicted.
- All 38 plants byte-identical on both fidelities. Release property tests
  223/223; the Godot binding did not change. Left open (E28): a restart asks no
  trip that does not hold the equipment.

**M41 is CLOSED (2026-10-05): no restart onto burst tubes, and what a loop
watches — `docs/DEFERRED.md` E28's tube clause struck (the rest open), F3 struck.**
The user's DECISION, on M40's two findings: "yes, fix both". DESIGN §46.
- **No restart onto burst tubes** (M41.0): when the last trip holding a furnace
  lets go and its reset would restart it, tubes that have burst — or stand at or
  past their limit, so the next tick bursts them (a fire heats a dark coil) —
  refuse the restart. The trip re-arms; the furnace stays dark, its loop in
  MANUAL, for a person. Checked in `release_equipment`, so a relight through an
  AUTO loop is caught too. `FurnaceTubes::limit_reached` owns the comparison.
- Three gates, `tests/restart_tubes_reference.rs` (burst on the trip's tick 75 and
  dark past the re-arm at 128; a patched, reset furnace dark until new tubes and
  a person's AUTO; a coil fired past 100 °C between ticks not relit, and relit
  one tick earlier). Four mutations, all caught. All 38 plants byte-identical.
- **`ControlSnapshot::watches`** (M41.1): `{"node":…}` or `{"pipe":…}`, always
  written. The eleven loop plants moved their fingerprints on both fidelities,
  wire only: with the field deleted every snapshot is byte-identical. The furnace
  screen finds its loop by it; its three stories print the same lines.
- Left for the user (E28): tubes replaced during a stop are relit by the trip, as
  agreed ("no restart onto burst tubes"), and a restart asks no trip that does not
  hold the equipment. Release property tests 223/223; Godot feature build and
  clippy clean.

**M40 is CLOSED (2026-10-05): what a trip watches, and a reset that restarts —
`docs/DEFERRED.md` F2 struck, E15's restart clause struck, new rows E28 and F3.**
The user's DECISION, on M39's two findings: "add the option to autoreset, when and
where applicable" and "make it know"; then, told the two meanings of auto-reset
and the real-plant rule, "make like in a real plant as default". DESIGN §45.
- **`TripSnapshot::watches`** (M40.0): `{"node":…}`, `{"pipe":…}` or
  `{"coil":…}`, always written. The five trip plants moved their fingerprints on
  both fidelities, wire only: with the field deleted every snapshot is
  byte-identical. The furnace screen marks each trip on the gauge it watches.
- **`reset = "manual" | "manual_restart" | "auto"`** (M40.1). Absent is
  `manual`, the real-plant rule and the old behaviour: a person resets, nothing
  restarts. `auto` re-arms past a required `reset_limit_*` strictly on the safe
  side. A restart hands the equipment back as before the STOP (a per-equipment
  record): an AUTO loop through the bumpless transfer, else the old pump, opening
  or duty — only if every trip that held it allows it and none was pressed.
- **Demo `furnace_coil_trip_autoreset.toml`** (38th file): cuts and relights every
  139 ticks, 43 trips in 6 000, on both fidelities; the fouled coil cannot settle
  on its 60 °C target. Key 3 on the furnace screen, its timeline gated.
- Eleven gates, `tests/trip_reset_reference.rs`; eleven mutations, ten caught,
  pass 3's order equivalent as predicted. M40.1 moved no plant; release property
  tests 223/223; Godot feature build and clippy clean.

**M39 is CLOSED (2026-10-05): the furnace screen — a Godot scene for M34–M38.**
The user's DECISION, from four directions offered with nothing past its trigger:
the recommendation. No engine, loader, binding or scenario change. ROADMAP M39;
DESIGN §44 (no interface change; it records the scene's rules).
- **`demo/furnace.tscn`** draws `furnace_coil_trip` (key 1) or `furnace_burnout`
  (key 2): flames sized by the duty and by `tube_fire_w`, the coil coloured by its
  temperature and broken when burst, a coil thermometer marked with the burst
  limit and the flame, the leak, and a panel of the furnace's books, each trip's
  reading against its limit (`by hand` when pressed) and the loop. Refusals show
  in the engine's words. E presses every armed trip, R resets, A/W/S drive the
  loop, Up/Down the fuel, P patches, N fits new tubes. Nothing physical computed.
- **`--auto` runs** both stories headless: the trip at 75, again at 238 on the
  same target, held on 50 °C, pressed at 900, reset at 901 and dark until 950; the
  burst at 1 145 (0.815 kg/s, 34.89 MW), new tubes refused at 1 300 and fitted at
  2 500. Both match their scenario headers' CLI figures.
- **Gated**: `crates/godot-ext/tests/furnace_screen.rs` replays both timelines
  through `bridge::Session` with the scene's command text byte for byte; a float
  id (`0.0`, what GDScript reads) fails it, which is why the scene casts.
- **Found, not built**: a reset does not relight the furnace (E15, now visible on
  screen); a trip's snapshot does not say what it watches (new row F2).
- `run/main_scene` unchanged; no corpus can move.

**M38 is CLOSED (2026-10-04): a trip pressed by hand, the emergency-stop button —
`docs/DEFERRED.md` E15's manual-trip clause struck, the rest of E15 open.** The
user's DECISION ("a relatively small batch"), told first that E15's trigger has not
fired: the Godot demo shows no trip, so this is groundwork. DESIGN §43.
- **`Command::ManualTrip { trip_id }`** fires one ARMED trip AT the command: it
  latches, writes every action's safe state and hands the loops on that equipment
  to MANUAL — the trip pass's own write, factored out as `write_trip_actions` — so
  every refusal holds from the moment it lands, not from the next tick.
- **`TripState::Tripped` gains `by_hand`**, written only when true: a measured trip
  serializes as M22's form, so no fingerprint moved. A press records the NEXT tick,
  the first to run in the safe state, as a measured trip does.
- **The reset is unchanged**: a press on a healthy plant resets at once, one past
  its limit cannot. Refused on a tripped trip and an unknown id; a trip whose
  measurement does not exist yet (a furnace outlet before tick 1) can be pressed —
  and its reset is refused until a tick has run (found by review: it was an
  engine fault, gate 8, fixed in its own commit).
- No loader key, no scenario file. Eight gates, `tests/manual_trip_reference.rs`;
  the bridge's sweeps gained `manual_trip`. Seven mutations, all caught.
- **All 37 plants byte-identical on both fidelities**; Godot feature clippy clean.

**M37 is CLOSED (2026-10-03): a furnace's tube burn-out — `docs/DEFERRED.md` B40's
burn-out clause struck (B40 narrows to the commanded fire), new rows B41–B44.**
The user's DECISIONS, each against the recommendation: keys on the furnace, on EVERY
furnace, the fire FED BY THE LEAK, patch-then-reset, gas holes built, round
game-tuning limits. DESIGN §41 (gas holes) and §42 (the burn-out).
- **M37.0, a hole in a gas line** (§41): the isentropic nozzle law with the liquid
  `Cd`, its choke derived from `γ`; both of M6.1's refusals removed. 0.104 554 kg/s
  choked through 1 cm² at 10 bara methane, against a 30-digit hand calculation.
  No plant moved.
- **The burn-out** (§42): `FurnaceTubes` on every furnace; the loader splits every
  furnace's outlet for a dormant hole. A coil at or past `tube_failure_c` bursts the
  tubes on the next tick's top pass (latched); the leak burns as FUEL through the
  flame law (`tube_fire_w`); `PuncturePipe` at zero patches, `ReplaceTubes` re-arms.
- **Shipped values**: 550 °C (850 °C on the gas drum, which settles at 727.4 °C),
  1 cm², heating values crude 42.686 (GREET), gas oil 42.8 and methane 50.0
  (Engineering ToolBox) MJ/kg, water 0. Round game values, not API 530 (B43).
- **Corrections it forced**: a column's FEED takes a hole (only a draw is refused);
  a split pipe is still the declared pipe for one-hop signs; a furnace outlet
  cannot be metered. The gas drum settles 1.43 K hotter (two gas segments), kept.
- **Demo `furnace_burnout.toml`** (37th file): bursts on tick 1 145, a 34.9 MW fire,
  the coil at 1 781.2 °C under its flame. The dry-fired demo bursts at 1 766, dry.
- 21 plants byte-identical, 15 moved, 1 new. Fourteen mutations, thirteen caught and the strict comparison uncaught as predicted.

**M36 is CLOSED (2026-10-03): a furnace's flame ceiling — `docs/DEFERRED.md` B40's
ceiling clause struck, the burn-out clause open.** The user's DECISION: both of B40's
remedies, the ceiling first, on every furnace. DESIGN §40.
- **The duty is a FIRING rate now.** The coil absorbs `Q·(T_f − T_c)/(T_f − T_a)`
  (a well-stirred firebox whose flue leaves at the coil's temperature); the rest is
  the stack loss, published as `NodeSnapshot::flue_loss_w`. The step stays exact
  (a second conductance in M34's form); an unlit furnace is M34's arithmetic bit
  for bit; a fire is not fuel and goes into the metal whole.
- **`flame_temperature_c`**, required, no default, above the 20 °C air. Every
  shipped furnace: 1 951.1 °C, methane in air at the stoichiometric ratio
  (Marzouk 2023, doi:10.48084/etasr.6132). Coils re-settled at load by the new law.
- **Five duties re-derived** for their design targets (the four cascade crude plants
  2.1051 → 2.2607 MW, `fired_gas_drum` 0.56 → 0.8837 MW); the rest run cooler.
- **Demo `furnace_dry_fired.toml`** (36th file): M33's plant on a tenth of the charge,
  no trip. Dry from tick 1 749; the coil reaches 1 794.8 °C at 6 000 and 1 950.99 °C at
  20 000, under its flame, on both fidelities.
- 21 plants byte-identical, 14 moved, only `fired_gas_drum`'s iterations moved. M32's
  trip now fires at 1 355 (was 1 251), M35's at 75 (was 72), M33's still at 3 017.
  Seven mutations, all caught; Godot feature build and clippy clean.

**M35 is CLOSED (2026-10-03): a trip on a furnace's coil and outlet — `docs/DEFERRED.md`
E13 narrowed to the cooler's outlet.** The user's DECISION, with M34's. DESIGN §39.
- **`{ coil = "…", variable = "temperature" }`**: a new measurement POINT, the
  coil state M34 writes every tick; present from load, so compared from tick 1.
  Trips only — a loop on a coil (a skin override) is refused by name.
- **A furnace's OUTLET** takes M33's flow rule (blind on tick 1's pass only); the
  exemption grows by exactly that case. A cooler's outlet stays refused.
- **Demo `furnace_coil_trip.toml`** (35th file): the M19 loop on a FOULED coil. The
  100 °C skin trip cuts at tick 72 with the outlet at 55.53 °C; the 70 °C outlet trip
  never fires; the untripped twin holds 60 °C with its tubes at 117.3 °C.
- Six gates, `tests/coil_trip_reference.rs`; seven mutations, six caught. All 34
  earlier plants byte-identical. M34's coil and its E19 fix: docs/MILESTONES.md.

**M34 is CLOSED (2026-10-03): a furnace's tube coil — `docs/DEFERRED.md` row B39
struck, E10 narrowed to coolers, new row B40.** Taken on a DECISION (the user's):
every furnace gets one, knowing it moves thirteen plants. One slice: DESIGN §37.
- **`FurnaceCoil`** on `NodeKind::Furnace`: heat capacity, metal-to-process `UA`,
  and a temperature that is a STATE (written back at the end of every tick). Duty
  and fire go into the metal; the fluid takes `G·(T_c − T_in)`,
  `G = W·(1 − e^(−UA/W))`. Exact step (never divides by `G`), and the fluid gets
  `Q − C·ΔT_c/dt` from the STORED change, so the first law closes at the furnace.
- **Three required keys**, no defaults: `coil_heat_capacity_mj_per_k`,
  `coil_ua_kw_per_k`, `coil_temperature_c`. Shipped coils: 1 MJ/K per rated MW,
  `UA` = 2 × the load capacity rate, loaded at the steady coil on TICK 2's flow.
- **A furnace is never `held`**: with no flow the fluid reads the coil. Outlet
  loops act on a stagnant coil; a stall no longer opens a furnace cascade; the
  held-outlet rule survives for COOLERS and is gated there. Outlet trips stay
  refused, each unit for its own reason (M35 builds the furnace's).
- **21 plants byte-identical, 13 moved**; only `fired_gas_drum`'s iterations moved
  (4 571 → 13 074 Newton). Steady states unchanged. M32's trip now clears at tick
  1 294, not inside its tripping tick; M33's twin reads 419 °C at 5 000, not 1 521.
- **The coil reached E19's trigger** (a cascade furnace at full fire dips a hair
  off its limit for 1–3 ticks); closed by a saturation latch, DESIGN §38.

**M33 is CLOSED (2026-10-03): a trip on a pipe's flow, the low-flow furnace trip —
`docs/DEFERRED.md` row E13's flow clause, struck; the outlet clause stays open.**
Taken on a DECISION (the user's, on gameplay grounds). One slice: DESIGN §36.
- **`{ pipe = "…", variable = "flow" }`** on any declared pipe, `limit_kg_per_s`
  of any finite sign (at or below zero is a reverse-flow trip).
- **The missing measurement**: a flow is read from the last solve, so it is absent
  for exactly ONE trip pass, tick 1's; the trip stays armed and compares nothing
  on it. Every other absence is still an engine fault. A plant loaded lit on too
  little flow therefore fires tick 1 unprotected and trips on tick 2 (gated).
- **No startup bypass** (stays E15): a cold start trips on tick 2 with nothing to
  cut, and the latch is the start permissive. An OUTLET trip stays refused: it is
  absent again whenever the unit stagnates, which is the low-flow condition.
- **Demo `scenarios/furnace_low_flow_trip.toml`** (the thirty-fourth file): a tank
  drains by gravity through a furnace fired at 0.6 MW; the 2 kg/s trip cuts at
  tick 3 017 on both fidelities, its twin to the bit until then. The cut moves no
  flow, so the demo cannot show the latch; a restorable-feed fixture does. The
  twin's outlet runs away on the vanishing trickle (new row B39, ungated).
- **Gates** `tests/flow_trip_reference.rs` (six) plus seven sweep cases. Eight
  mutations, seven caught; widening the exemption is uncaught as predicted. All
  33 earlier plants byte-identical, no iteration count moved; no Godot build owed.

**M32 is CLOSED (2026-10-02): a trip that cuts a furnace's fuel — `docs/DEFERRED.md`
row E14's furnace clause, struck; the cooler clause stays open.** Taken on a
DECISION (the user's, on gameplay grounds). One slice, note and build together:
DESIGN §35.
- **`TripAction::CutFurnace`** (`actions = [{ furnace = "…" }]`, no `position`,
  refused if given): writes zero duty, the furnace's ONE safe state. The hold
  check fails a tick on a lit furnace a latched trip holds; `SetFurnaceDuty`
  above zero is refused while latched, zero admitted. `SetHeatInput` (a fire) is
  not fuel and stays admitted.
- **The trip forces MANUAL on EVERY action's equipment** now, not a valve's
  alone (its own commit, byte-neutral). A furnace loop yields; on a cascade the
  furnace's loop is the inner one, so the cut opens the cascade with no new rule.
- **A cooler is still refused, for its own reason**: cutting it is losing
  cooling, the hazard rather than the protection. E14 stays open on that clause.
- **Demo `scenarios/tank_overheat_trip.toml`** (the thirty-third file):
  `tank_temperature_heating.toml` without its loop, 3 MW by hand, a 75 °C trip.
  Cuts at tick 1 251 on both fidelities, bit-identical to its twin until then,
  clears its condition inside the tripping tick (74.967 °C), 40.36 °C at 6 000
  against the twin's 89.77. At zero duty the furnace's outlet IS its inlet
  stream's temperature, bit for bit; the next pipe is 0.0008 K warmer (friction).
- **Gates** `tests/furnace_trip_reference.rs` (six) plus seven sweep cases in
  `trip_reference.rs`. A cascade hand-back test must move the outer target while
  the cascade is OPEN: moved in the closing batch it lands a 13.2 K proportional
  kick (M25's), which reads like a bump and is not the trip's. Seven mutations,
  six caught, the hold check's deletion uncaught as predicted. All 32 earlier
  plants byte-identical, no iteration count moved; no Godot build owed.

**M31 is CLOSED (2026-10-02): a check valve in gas service — `docs/DEFERRED.md`
row E24, now struck.** Taken on a DECISION (the user's). One slice, note and build
together: DESIGN §34. Read its "Corrections from building it" before touching
`fold_gas_service` or the random arm's choke detector.
- **`NodeKind::CheckValve::x_t`** (skipped when `None`, so no liquid plant's
  bytes move) under `Valve`'s rule in `require_gas_valve_x_t`: required in gas,
  refused in liquid, inside (0, 1). The shaped-heat-capacity refusal (B23)
  covers the disc. E24's "a disc that reads its own drop" was not a coupling:
  the opening is read off the branch's drive first, the fold splits it after.
- **`network::fold_gas_service` is the one owner of the compressible law** for
  all three valve kinds, the gas-without-`x_t` door included. Moved without
  reordering a single operation: all 31 earlier plants byte-identical on both
  fidelities, worst and total iterations unchanged.
- **Demo `scenarios/gas_receiver_check_valve.toml`** (the thirty-second file):
  `vessel_pressure_control.toml` without its loop, the receiver charged at 30 bar
  above a 20 bar header. Disc shut ticks 1–300; its twin blows 0.462 kg/s back
  into the header for 174 ticks; both settle at 16.49 bar. **It never chokes**
  (`x/x_choke` ≤ 0.0385) and BORROWS a globe valve's `x_t` (new row E27, no
  published `x_T` for a check valve). The fold is carried by
  `tests/gas_check_valve_reference.rs` (seven gates, a choked-in-band fixture
  among them) and `the_check_valve_arm_in_gas_service_...` in
  `solvers/tests/invariants.rs` (33 of 400 discs choked); the liquid arm is the
  same function with its draws unchanged.
- **Eight mutations, three escaped the first pass, all closed and re-run.**
  `valve_edge_is_choked` was one-sided and counted an UNFOLDED valve (flow above
  the plateau) as choked — now two-sided, no count moved. "The fold handed the
  drive, not the drop" was predicted inert and is not on a rising gas line (gate
  2's tail rises 30 m). An untested `skip_serializing_if` (gate 6 reads the
  serialized bytes). A choked valve's frozen Newton slope is entirely spurious
  (A18, re-measured): inside a choked disc's band the opening share is the whole
  true derivative. The Godot build and clippy were run and are clean.

**M30 is CLOSED (2026-10-02): a check valve — `docs/DEFERRED.md` row E22, now
struck.** Taken on a DECISION (the user's). One slice, note and build together:
DESIGN §33. Read its "Corrections from building it" before touching
`CompiledEdge::conductance`, `compile_edge`'s check arm or the game solver's sweep.
- **`NodeKind::CheckValve { cv_max, full_open }`** (`type = "check_valve"`, `kv`,
  `full_open_bar`, required, > 0): the relief valve's smoothstep read on the
  FORWARD DRIVE `S = dp − β` across its branch. Shut at `S ≤ 0` is exact (the
  flow has the sign of `S`), so reverse flow is exactly zero by `α = +∞`. `β` is
  load-bearing (an uphill outlet). Liquid only (gas refused at load and in
  `compile_edge`, E24); a loop, a trip and `SetValveOpening` each refuse a disc.
- **The opening's slope is a share of the CONDUCTANCE, on both solvers**, through
  one owner, `CompiledEdge::conductance` (`ṁ·k`, symmetric, `0.0` outside the band
  and added only when nonzero). Without it Newton diverges in the band (A14 again)
  and the game solver diverged with the pump running. The game solver also reads a
  disc FRESH inside its sweep (`fresh_check_edge`), which halved its band cost;
  what remains is row A19.
- **Demo `scenarios/tank_level_fill_check_valve.toml`** (the thirty-first file):
  M29's plant, a disc after the discharge line with 1 m of outlet, and a
  low-suction-level trip that stops the pump at tick 2 142. Shut on exactly every
  tick to 3 160, then reopens; its plain-valve twin runs backwards at 3.52 kg/s.
  **The tank is not kept**: higher while the disc is shut (2.27 m against 1.87 m
  at 3 000), lower at the end (0.947 m against 0.985 m). Gates are
  `tests/check_valve_reference.rs` (eight) plus a random arm in
  `solvers/tests/invariants.rs` (`the_check_valve_arm_shuts_lifts_and_conserves`,
  its own test so no recorded generator count moves). All 30 earlier plants byte-identical, no
  iteration count moved; a new `NodeKind` variant, so the Godot build and clippy
  were run and are clean.
- **Twenty mutations, eighteen caught**; the game solver's frozen read and its
  node step's share alone are uncaught by design (cost, not correctness). New rows
  E23 (cracking pressure), E24 (gas), E25 (restart surge into the pinned fill,
  24.80 against 6.90 kg/s), E26 (the disc reads its branch's drive), A19; A8
  re-measured (the shut stretch costs Newton 14–18 iterations a tick, and a plain
  valve shut there costs the same).

**M29 is CLOSED (2026-10-02): a fill valve holding a level — `docs/DEFERRED.md`
row E8, now struck.** Taken on a DECISION (the user's, on gameplay grounds). One
slice, note and build together: DESIGN §32.
- **The rule**: a level or pressure loop's valve is a DRAIN of its holdup (inlet
  pipe starts there: direct, the default) or a FILL (outlet pipe ends there: must
  SAY `action = "reverse"`), judged in one hop by `valve_side` in `build.rs`, the
  single owner — `cascade_pairing` calls it too. `check_holdup_valve_action` owns
  the table; a valve that is neither or both is refused either way.
- **The unchecked default was the larger hole**: before M29 a direct loop on a fill
  valve loaded and ran away. Refusing it (and direct on a valve two hops off)
  narrowed nothing any plant or fixture used — measured by the full suite.
- **Demo `scenarios/tank_level_fill_control.toml`** (the thirtieth file):
  `tank_level_control.toml` with the loop on the fill; parked, the two files are
  one plant bit for bit. A make-up valve on a vessel runs on a fixture. Gates are
  `tests/fill_valve_reference.rs`. All 29 earlier plants byte-identical, no
  iteration count moved; no snapshot change, so no Godot build owed.
- **Two new rows.** E21: a valve beyond one pipe (the walk). E22: a stopped pump
  drains the tank back through its fill, which the loop pins wide open.
- **A refusal case that "should not have loaded" and did** was a substitution
  landing in a header comment that quotes the same text: anchor swaps on the
  loop's own lines. And `let _ = f()?;` is not a mutation of `f()?;`.

**M28 is CLOSED (2026-10-01): a cascade primary held by its secondary's limit —
`docs/DEFERRED.md` row E16, now struck.** Taken on a DECISION (the user's). One
slice, note and build together: DESIGN §31. Read its "Corrections from building it"
before touching `step_loop` or `inner_limit`.
- **The rule**: while a secondary's actuator sits EXACTLY at 0 or 1 (pass 1's
  start-of-tick sample), its primary may not move the secondary's setpoint further
  that way. Which way is the SECONDARY's action: a reverse secondary's output rises
  with its setpoint. A held primary is the open cascade's arm: writes nothing,
  faceplate tracks, memory back-calculated. No trait or snapshot change.
- **E16's named remedy (external reset feedback) was rejected**: it moves the demo,
  and it is M25.1's mutation 2. "Bounded" understated the cost, which grows with
  the outer range (1.3 MW furnace: 0.39 K overshoot at 40–65 °C, 1.50 K at 40–90;
  held, 0.004 K at any range).
- **No shipped plant reaches it**; the four gates (`tests/inner_limit_reference.rs`)
  run on fixtures derived in-test, one per row of the direction table plus the
  release. All 29 plants byte-identical, no iteration count moved. Six mutations,
  six caught.
- **Two new rows.** E19: the exact test LEAKS on a drifting plant (a hair off the
  limit frees the primary for a tick; the hand model, at constant flow, could not
  see it). E20: a sustained limit parks the memory where a later step dips deeper.

**M27 is CLOSED (2026-10-01): two small fixes — `docs/DEFERRED.md` rows F1 and
B37, both struck.** Taken on a DECISION (the user's: "the small fixes first, then
E16"); neither was past its trigger. No separate design section: each is a
correction to the note that deferred it (DESIGN §8, §28 fork 6).
- **M27.1: `Engine::apply` refuses an unknown node or edge id** as
  `InvalidCommand` (`check_command_ids`, wildcard-free over `Command`, run before
  any arm) instead of panicking. The Godot bridge KEEPS its own guard: it is what
  gives a stale id the `unknown_id` code rather than `invalid_command`. The old
  characterization test is now `core_refuses_an_out_of_range_id`.
- **M27.2: `NodeSnapshot::running_dry`**, true on a tank the last solve starved,
  skipped when false. The solve's verdict, NOT "mass reads zero": an empty tank
  nothing draws on is not flagged, and that gate is the only one a mass-based flag
  fails. Only `tank_runs_dry` moves (both fidelities, the key only); "runs
  byte-identical" means post-M27.2 identical for it.

**M26 is CLOSED (2026-10-01): Newton's missing relief slope — `docs/DEFERRED.md`
row A14, now struck.** Taken on a DECISION (the user's). **M26.0** wrote DESIGN §30
(five forks, six gates, seven mutations, no code); **M26.1 landed 2026-10-01** and
built it. Read §30's "Corrections from building it (M26.1)" before touching
`assemble`, `compile_edge`'s valve arm or `CompiledEdge`. Nothing is past its
trigger; the next milestone is chosen from `docs/DEFERRED.md`. Five things to know.
- **A14's "two relieving vessels" was wrong.** A PSV's opening is frozen into its
  compiled branch, so `flow_ddp` holds it fixed and Newton's Jacobian never saw the
  valve open wider. On a vessel at a long timestep that share is as large as
  everything else holding the vessel, so each step overshot by about a whole step
  (ratio −0.855) and Armijo accepted it. One relief plant crawled 20 of 50
  iterations through its lift and was recorded as unaffected.
- **`CompiledEdge::relief_opening_log_slope`** is `d ln ṁ/dP_src` through the
  opening, set for a relief valve only (exact for liquid; the gas fold's
  `(x/x_s)/Y²` held fixed); Newton adds `ṁ·k` to the SOURCE column, so `J` is not
  symmetric in a band. Exactly `+0.0` outside the band and when the opening snaps
  shut (where it would be `0·∞/0`); a non-finite value is an `Err`. The game
  solver never reads it.
- **Measured**: 27 of 29 plants byte-identical on Newton with worst and total
  iterations unchanged, all 29 on `simple`; `relief_blowdown` and
  `relief_twin_vessels` move by at most 1.4e-7 from their first lift, total
  iterations 18 562 → 7 287 and 20 782 → 7 823. "Runs byte-identical" means
  post-M26.1 identical for those two on Newton. The twin runs at `dt = 1.0` (a
  fixture; the file stays at 0.1). Random spur trees Newton gives up on: 39 → 5.
- **Gate 1 differences in the SET pressure, not the inlet pressure**: the inlet
  pressure also moves the gas density and the fold (row A18, omitted on purpose,
  4% of the slope near full lift). Gas within 1.7e-3, liquid within 7.6e-9.
- **Seven mutations, six caught, one inert as predicted.** Dropping or flipping
  the term in `assemble` is invisible to gate 1 (it tests the field) and caught by
  the plant gates. The spur-tree bound in `invariants.rs` was tightened 50 → 15
  because the unfixed solver (39) passed it. Gates are
  `tests/relief_slope_reference.rs`.

**M25 is CLOSED (2026-10-01): cascade control — `docs/DEFERRED.md` row E2, now
struck.** Taken on a DECISION (the user's). **M25.0** wrote DESIGN §29 (eight forks,
eleven gates, thirteen mutations, no code); **M25.1 landed 2026-10-01** and built it.
Read §29's "Corrections from building it (M25.1)" before touching
`ControlLoop::actuator`, `run_control_loops` or `link_cascades`. Nothing is past its
trigger; the next milestone is chosen from `docs/DEFERRED.md`. Six things to know.
- **A primary declares `actuator = { loop = "…" }`** (`Actuator::Loop`), and its
  output is the secondary's setpoint as a fraction of `range_min_*`/`range_max_*`
  (`SetpointRange`, the secondary's unit, both `_c` keys `+273.15`, both ends through
  `check_setpoint`, `min < max` after conversion). `actuator_position`/
  `set_actuator_position` read and write it, so tracking, bumpless transfer and
  anti-windup reuse existing code. **The write is clamped to the range**, or the
  round trip can read `1.0000000000000002` at the clamp; only a unit test defends it
  (the demo's `40 + 1·25` rounds exactly). A flow range cannot start at 0.
- **Pass 2 runs in two halves**: primaries first, their setpoint writes applied, then
  every other loop (`step_loop`). Two levels only, refused by `link_cascades` after
  every loop is built (a file may declare the secondary first). **Deleting the depth
  rule is caught only by its message**: every admitted pairing puts the primary on a
  tank, so the pairing rule's holdup-inner refusal also refuses every chain; E18 is
  where the depth rule becomes load-bearing.
- **Open = the secondary will not act this tick** (not AUTO, by a human or a trip, or
  nothing to measure: tick 1, a stagnant outlet). An open primary writes nothing,
  tracks, and re-seeds. A stall is seen ONE tick after the flow stops (a loop reads
  last tick's outlet), so a setpoint raised in the same command batch gets one
  closed tick and the proportional kick (gate 11 raises it a tick later).
- **Two refusals beyond the note**: a secondary declared outside its primary's range
  (tick 1 is open and would fail the re-seed), and `SetSetpoint` outside the range
  under a MANUAL primary (`check_loop_owned_duty`'s rule). Every primary must DECLARE
  its action, a drain's included.
- **Measured**: all twenty-eight earlier plants byte-identical on both fidelities,
  worst AND total iterations unchanged ("runs byte-identical" means post-M25.1,
  unchanged). The demo matches the hand model: settles from tick 2 579 (2 578),
  outlet ≤ 64.9925 °C, outer clamp 69 ticks, heater fire +0.0753 K, tank fire back
  inside +2 006, closing −0.0039 K (1.366 K with the memory untouched). Seventeen
  mutations, sixteen caught, #12 inert as predicted. Godot build and clippy clean.
- **Demo `scenarios/furnace_cascade_control.toml`** (the twenty-ninth file):
  `tank_temperature_heating.toml`'s plant, the outlet loop inside, a tank loop outside
  over 40–65 °C. Gates are `tests/cascade_control_reference.rs`; the level pairing
  (drain and fill) and the cooler pairing run on fixtures there.
  `ControlSnapshot::drives`, skipped when `None`.

**M23 is CLOSED (2026-09-30): tank overflow — `docs/DEFERRED.md` row B28, now
struck.** Taken on a DECISION (the user's), and past its trigger all along: run
past the corpus's 6 000 ticks and six shipped plants overflow, the first at 11.7
simulated minutes. **M23.0** wrote DESIGN §27 (seven forks, ten gates, twelve
mutations, no code); **M23.1 landed 2026-09-30**, after M24.1, and built it. Read
§27's "Corrections from building it (M23.1)" before touching a tank's update,
`LeakRole` or the edges the loader builds. Nothing is past its trigger; the
next milestone is chosen from `docs/DEFERRED.md`. Six things to know.
- **Every tank owns an overflow**: a loader-built, engine-written edge
  `<tank>__overflow`, `LeakRole::Overflow { owner }`, to the plant's first
  `Atmosphere` or a new `overflow_atmosphere`, built AFTER the vents (gate 9). At
  the end of each tick, AFTER the boil-off, anything above `TankState::capacity`
  (`ρ(x_end)·A·H`) leaves through it at the tank's end-of-tick state, and the
  tank's temperature and composition are not recomputed. "Full" is a MASS
  comparison through `TankState::mass_at_level`, which the loader's initial
  inventory also calls, so a tank declared full ties exactly.
- **`LeakRole::is_engine_written()` is the solve-and-pass skip** (vent OR
  overflow, four sites); `is_boiloff_vent()` now means the vapour path only, and
  `overflow_owner()` is the single owner of "whose spill is this". A tank over its
  brim with no overflow of its own is an `Err` (the note left it open), and
  `PuncturePipe` refuses an overflow by its own message. **A frontend finds the
  spill by NAME** — `EdgeSnapshot` has no role — so a declared pipe named
  `<tank>__overflow` is refused. Geometry is checked at load: non-positive area or
  height, a negative level, a level over the brim.
- **Bytes as premise 2 measured, on one more plant**: seventeen pre-M23 plants
  byte-identical on Newton, sixteen on the game solver; the eleven that gain a new
  atmosphere (premise 2's ten plus `tank_runs_dry`) move by at most 4.9e-11
  (Newton) / 1.7e-8 (game). **"No iteration count moved" was false**: the new node
  moves the cold seed, which only tick 1 uses — +1 on six Newton and three game
  plants, −2 on `tank_runs_dry`. The probe sampled every 10 ticks and could not
  see it; compare the corpus's `total_iterations`. From here "runs
  byte-identical" means post-M23.1 identical.
- **M24.1's "worst dry tick 10 (Newton)" was that same tick-1 cold solve**; the
  worst DRY tick is 7 (Newton) / 8 (game), corrected in B38. And the M20.1
  shut-valve control (the inlet reads a nonzero residual) was the seed's, not the
  valve's: it is now taken over six supply levels (outlet 0.0 on all, inlet
  nonzero on two). M20.1's inlet-for-outlet catch survives (5 877 of 6 000 ticks).
- **Demo `scenarios/tank_overflow.toml`** (the twenty-eighth file) is
  `tank_overfill_trip.toml` without its trip, asserted byte for byte in
  `trip_demo.rs`: brim at tick 2 859, then exactly full, 19 454.49 kg spilled by
  tick 6 000 on both fidelities. Gates 1–9 are `tests/overflow_reference.rs`.
- **The mutations: fifteen edits, twelve caught, three inert** (1, 6, 12). Six
  predictions were wrong. The note's mutation 1 spills exactly 0.0 as written
  (the excess is still a mass difference); 1b, the spill in level terms, spills
  4.4e-12 kg/s forever and is caught. A rate cannot go stale (mutation 6) because
  the solve writes zero onto every engine-written edge each tick. **Two escaped
  until a gate was added, because the demo is water only**: the spill's own
  composition is now asserted on a two-liquid fixture.

**M24 is CLOSED (2026-09-30): a tank that runs dry — `docs/DEFERRED.md` row B29,
now struck.** Taken on the user's instruction while M23.1 was unbuilt; M23.1 landed
second and made the pointer fixes DESIGN §28 lists (M24.1 had already added pointers
in §27). **M24.0** wrote DESIGN §28 (seven forks, ten gates, eleven mutations, no code);
**M24.1 landed 2026-09-30** and built it. Read §28's "Corrections from building it
(M24.1)" before touching `network::solve_with_active_anchoring`, `Capacitance`, the
sweep's starved-tank branch or a holdup's mass update. Six things to know.
- **A starved tank is a second active set in the shared driver**: a FREE node
  supplying `m/dt` through `network::accumulation` (`Capacitance::Starved`, not an
  anchor — `base_anchors` excludes it). It starves on `q_out·dt > m` from a converged
  pass and recovers when its solved pressure exceeds the pressure its level pins.
  Every solve starts all-wet; one loop and one budget over (anchored, starved) pairs;
  warm start committed once, from the accepted pass. The driver now takes `dt`.
- **"Anchoring settled" is judged against the pass's OWN anchors**, and the next
  pass's anchored set is built from the next classification's (a starved tank stops
  being an anchor). **A starvation-only repeat is accepted as the more starved pass;
  if it surfaces on the less starved one (two tanks), the union runs once more with
  recovery frozen.** A single tank cannot recover inside a solve except on the
  boundary, so the recovery rule and the union are gated on stub passes in
  `tests/invariants.rs`.
- **The solve reports each starved tank's supply and OWN residual, and each vessel's
  own residual** (`HydraulicSolution::starved`, `vessel_residual`; not in `Snapshot`,
  so no Godot build owed). `engine::checked_holdup_mass` replaces both silent
  `.max(0.0)` clamps: rounding (`ROUNDING_MASS_FRACTION` × gross traffic) plus that
  residual, else an `Err` naming the holdup. `Engine::last_solution()` is a new
  read-only test accessor. **Never bound anything by `SolveDiagnostics::residual`**
  (the plant's worst node, M9.2's defect).
- **The sweep resolves a starved tank as a mixing vertex** whose inventory is one
  more inflow (no `heat_load` there, row B35); its update debits outflow at that mix;
  below the thermal floor it takes the pass-through state. A recycle through a dry
  tank is refused by name (B34).
- **All twenty-six pre-M24 plants are byte-identical on both fidelities with no
  iteration count moved**, and the generated plants' reachability counts are
  unchanged; "runs byte-identical" means post-M24.1 identical, unchanged. Over 30 000
  ticks `tank_flow_control` now keeps its 179 640 kg to 1.7e-4 kg (Newton) / 1.2e-2 kg
  (game) — 200 149 kg were created before.
- **Demo `scenarios/tank_runs_dry.toml`** (the twenty-seventh file): diesel buffer tank
  fed kerosene, dries at tick 1 227 with 14.2 kg left and passes its feed through at
  −210 636 Pa from 1 228.
- **The mutation pass: eleven edits, ten caught**; carrying the starved set across
  ticks is uncaught by design and is new row B38 (a 14× cost saving nobody needs
  yet). Four predictions were wrong: in-tick recovery and the starvation-only
  repeat are reachable ONLY on stub passes (`tests/invariants.rs`), and the
  start-of-tick debit was first caught by accident (0.3 µg over the thermal floor)
  until gate 3c made it deliberate. `tank_overfill_trip` starves 1 215 ticks of
  12 500 on Newton and never repeats.

**M22 is CLOSED (2026-09-29): interlocks and trips — `docs/DEFERRED.md` row E6,
now struck.** Taken on a DECISION (the user's, on gameplay grounds). **M22.0**
wrote DESIGN §26 (eight forks, nine gates, sixteen mutations, no code) and
**M22.1 landed 2026-09-29** and built it — read §26's "Corrections from building
it (M22.1)" before touching trips. The next milestone is chosen from
`docs/DEFERRED.md`.

**What M22.1 found.** (i) All twenty-five pre-M22 plants are byte-identical on
both fidelities with no iteration count moved; "runs byte-identical" means
post-M22.1 identical, unchanged. `corpus --baseline` compares fingerprints only,
so iteration counts need their own comparison. (ii) **A trip that fires on a
crossing can clear its own condition INSIDE the tick that fires it** (it acts
before that tick's solve): exactly when its action reverses the measurement by
more than the one-tick overshoot within one tick. True on the demo (0.0006 m over,
0.0014 m back) and on one fixture, where the LATCH then holds the equipment, not
the condition. False on a plant loaded inside its condition or with a slow action,
which is where the reset's "condition still holds" refusal applies. (iii) `TripState` is tagged `status` (`{"status":"tripped","at_tick":…}`).
(iv) Nineteen mutations, eighteen caught; the hold check's deletion is uncaught on
purpose. (v) No shipped plant passes its own tank height (B28 swept; closest
0.859) — true only out to 6 000 ticks, and B28 is closed by M23.1. Setpoint and limit conversion share `declared_value` in `build.rs`.

The design as built: `PlantGraph::trips`, a plain struct (no trait),
`[[trips]]` with `direction = "high" | "low"`, `limit_m`/`limit_bar`/`limit_c`
converted at the setpoints' own site, and an `actions` list of `{ pump = … }` and
`{ valve = …, position = … }`. A trip fires at `≥`/`≤`, latches, writes its safe
state once, forces loops on its valves to MANUAL, runs BEFORE the loops, and is
re-armed by `Command::ResetTrip`, which is refused while the condition holds and
restarts nothing. Only quantities present from load may be watched (flow and
outlet trips are E13). **A stopped pump conducts** (it keeps its resistance), which
is why a trip takes a list of actions. Demo: `scenarios/tank_overfill_trip.toml`,
trips at tick 1 236; its untripped twin filled a 10 m tank to 15.17 m (B28: no
overflow) until M23.1, and now spills at its brim as `tank_overflow.toml`. `TripSnapshot::state` is `Armed | Tripped { at_tick }`, and a reset
clears the tick. **A level does not tie exactly at load** (4 of 8 declared values
read one ULP high), so the at-the-limit gate uses a vessel's pressure and a
tank's temperature, which do (8 of 8 each). The Godot bridge's `referent` has its
`Trip` arm, and the godot-feature build and clippy were run at M22.1 and are clean.

**M21 is CLOSED (2026-09-29): the game solver's stiff-pair stall — `docs/DEFERRED.md`
row A3, now struck.** Taken on a DECISION (the user's). **M21.0** wrote DESIGN §25
(five forks, eight gates, nine mutations, no code) and **M21.1 landed 2026-09-29**
and built it. Read §25's "Corrections from building it (M21.1)" before touching
`SimpleFlowSolver`. The next milestone is chosen from `docs/DEFERRED.md`.

**The mechanism.** A vessel and the zero-volume node across a wide line converge
TOGETHER at `c/(g + c)` per Gauss–Seidel sweep, where `g` is the line's conductance
and `c = C/dt`: 0.98850 predicted against 0.988502 measured. A3's old "5.4× under
the cap" was measured on `relief_blowdown.toml`, whose PSV inlet had been re-sized to
5 m × 60 mm to dodge this. One edit (2 m × 100 mm, or `dt = 1.0`) failed the game
fidelity. Newton was untouched by M21 — and was affected after all, under the
cap: `relief_blowdown` at `dt = 1.0` took 20 of 50 iterations and the twin failed
(A14). M26.1 fixed that; see DESIGN §30.

**The fix, in `SimpleFlowSolver`.** After each sweep, `correct_groups` shifts each
group of pressures by one common amount: a scalar Newton step on the group's net
imbalance over its BOUNDARY conductance plus its members' `C/dt` (Settari & Aziz
1973). The groups come from `build_groups`, a hierarchy of heaviest-edge pairs
with no threshold, built once per pass. Four rules, each defended by a test:
- **Groups of one are excluded.** A plant with no two neighbouring unknowns builds
  no level at all, which is why fifteen plants are byte-identical.
- **A group whose members all meet `network::meets_node_bar` is skipped.** This is
  the stopping test itself, split out of `grade_nodes` so it is the same code.
- **The group step's Armijo trial RECOMPILES its boundary edges.** On frozen
  coefficients it cycles inside a PSV's band.
- **Both steps share one ladder, `armijo_step`**, which returns exactly `0.0` when
  every trial is refused.

**What it measured.**
- It reproduces the out-of-repo prototype bit for bit on 35 plants.
- `relief_blowdown`: 920 → 8 sweeps.
- Every plant one edit from the cap runs and agrees with Newton to 4.72e-5 on
  every node pressure at every snapshot.
- Newton: byte-identical on all 24 pre-M21 plants.
- `simple`: fifteen plants byte-identical, and the other nine
  (`cavitating_pump`, `fcc_plant`, `fired_gas_drum`, `leaking_line`,
  `relief_blowdown`, `tank_flow_control`, `tank_level_control`, `tank_pump_valve`,
  `vessel_pressure_control`) move by at most 3.75e-6. **From here, "runs
  byte-identical" means post-M21.1 identical for those nine on `simple`.**
- Generated plants the game solver solves: gas chains 187 → 198/205, spur trees
  191 → 264/305.

**`scenarios/relief_twin_vessels.toml` is new** (two stiff pairs in one plant). It
declares `flow = "newton"`, against §25, because CI runs each file as declared
plus `--solver simple`; a file declaring `simple` is never run under Newton. The
old solver, and the one-group-per-connected-set shortcut, both fail it at tick 1,
so CI's corpus defends the hierarchy.

**Four findings to carry forward.**
- §25's gate 6, written at the rounding floor, is ESTIMATED to be blind to its own
  mutation on the relief plants' numbers: the wrongly written step comes out below
  one ULP of a pressure. This was estimated, not run, and a weak-boundary liquid
  group could differ. It was moved to a test on the shared ladder, which is
  measured to be the only thing catching it. Estimate the size of what a mutation
  changes before writing its gate.
- Mutation 8 (skipping on `tol_abs` alone) moved nine plants and failed no test
  until a unit test was added for it.
- Mutation 7 (simultaneous shifts within a level) is inert and uncaught: new row
  A16. The group step's baseline and trials come from two compiles on a gas edge:
  new row A17.
- The mutation harness's restore silently turned LF into CRLF (Python text mode on
  Windows), and a checksum read back in text mode passed. **Write sources with
  `newline=""` and check with `git diff`/`file`, not with a read through the same
  translation.**

`scenarios/` holds **twenty-six** files (M22.1 added `tank_overfill_trip.toml`).

**M20 is CLOSED (2026-09-29): the fourth controlled variable, FLOW — a valve
holding the flow in one of its own two pipes.** Taken on a DECISION (the user's,
2026-09-29, on the grounds of M17–M19): the commonest loop in a refinery and the
inner half of a cascade. **M20.0** wrote DESIGN §24 (seven forks, eight gates, ten
mutations, no code); **M20.1 landed 2026-09-29** and built it — read §24's
"Corrections from building it (M20.1)" before touching `measure`,
`MeasurementPoint` or `build_controls`. E1b now holds only a bubble-point-bounded
temperature setpoint; the next milestone is chosen from `docs/DEFERRED.md`.

**What M20.1 found building it.** (i) **All twenty-three pre-M20 plants are
byte-identical on both fidelities with no iteration count moved**; "runs
byte-identical" means post-M20.1 identical, unchanged. (ii) **The hand
simulation held**: within 0.01 kg/s from tick 137, twin at 10.207091 kg/s,
`K·G = 0.535` measured locally. (iii) **"Pins within ten ticks" was wrong — 31**:
a PI loop at half its bound climbs onto a clamp along its SLOW pole, and near a
clamp it dips off for exactly one tick by `K·(e − e₊)` (M18's (iv), now asserted
to the bit). The release decays at 0.9645 per tick, which is that pole, and the
gate asserts the band the pole spans. (iv) **"Exactly 0.024" is
`0.023999999999999994`**: a gate against a hand formula compares against the
controller's own order of operations, not a literal. (v) **All fourteen
mutations are caught** (§24's ten plus four), each read for why it fired. Reading
the pipe's stored flow instead of the solve is visible ONLY at load, because
step 2 of the tick copies the solve into that field, which is the premise
measured. The inlet-for-outlet identity mutation is caught by any tolerance
below the valve's own solver residual: still at 1e-12, not at 1e-9, measured
after a first write-up wrongly said only bit equality would do. `scenarios/` holds **twenty-four** files. The five
things §24 settled:

**A pipe's flow is stored AND absent at load.** `Pipe::stream.mass_flow` is on the
graph, but the loader writes `Stream::stagnant`'s zero there — an initialiser, not
a declaration. So M19's rule is reused whole (no measurement, no action; the PI
memory stays PENDING), and `measure` reads `last_solution` (`None` at load), never
the pipe's stream. **A loop measures at a `MeasurementPoint { Node, Pipe }`**, and
the file says `measurement = { pipe = "…", variable = "flow" }`, looked up among
DECLARED pipes only. A pipe with `leak_to` is refused.

**The first reverse loop on a valve.** A flow loop's actuator is a valve, its
measured pipe must be one of that valve's two edges (`validate_degrees` makes that
one hop), and it must say `action = "reverse"`. Absent or `"direct"` is refused.
**E8 still refuses reverse on a level or pressure loop's valve.** A meter away from
its valve and a bypass valve are E12; reverse flow, which pins the valve open, is
E11.

**Meter the valve's OUTLET.** A device folds into its outlet edge, so a shut
valve's outlet pipe reads exactly `0.0` and its inlet `−1.547e-11`. Zero flow is a
real measurement (no `held` set), and the demo's shut-start gate needs the exact
zero. **The stability bound is `K·G < 2/(2 − dt/T_i)`** — 1.053 at `T_i = 10 s`;
M19's "`K·G < 1`" is the `T_i → ∞` limit, and M19 measured at 600 s, so nothing
was wrong. Keys `setpoint_kg_per_s` and `gain_per_kg_per_s`, with no unit
conversion anywhere. Demo `scenarios/tank_flow_control.toml` =
`tank_pump_valve.toml` with `dt = 1.0`, valve at 0.4, and a loop on `fill_line`
holding 12 kg/s at `K = 0.02`, `T_i = 10`. The godot-feature build and
clippy were run at M21.0 and are clean.

**M19 is CLOSED (2026-09-24): a furnace holding its own OUTLET — the zero-volume
measurement, what `docs/DEFERRED.md` row E1b has left of temperature.** Taken on a
DECISION (the user's, on gameplay grounds, as M17 and M18). Two slices: **M19.0**
(DESIGN §23, seven forks, eight gates, ten mutations, no code) and **M19.1**,
which built it — read §23's "Corrections from building it (M19.1)" before
touching `measure`, `ControlLoop` or `PiController`. E1b keeps flow control; the
next milestone is chosen from `docs/DEFERRED.md`. Four things the note settles.

**The premise is TRUE this time**: an outlet is resolved into `NodeStates`, has no
field on the graph, and is absent before the first tick. The rule is **no
measurement, no action** — the loop holds its actuator, the faceplate tracks, and
a PI loop's memory is PENDING until its first measurement, where the ordinary
back-calculation seeds it. `measure` takes the resolved states (one owner kept);
`last_measurement` and `ControlSnapshot::measurement` become `Option`, skipped when
absent — byte-neutral for every existing loop, which is exactly why the corpus
cannot defend it and gate 1 asserts it on the demo's bytes. MANUAL→AUTO with
nothing to measure is refused.

**A stagnant outlet is the same state**: the no-inflow fallback (the last value,
or ambient) is a placeholder, so the sweep records which nodes took it and
`measure` returns nothing for them. Only a furnace and a cooler are admitted
(others → new row E9); the junction-PRESSURE refusal's reason ("no tick-0 rule")
expires and is reworded.

**The gain bound is a STABILITY bound**: no thermal mass, so poles `1` and `−K·G`,
stable only for `K·G < 1` (new row E10). `G = 33.28 K` per unit output on the
demo; the M18 tuning moved to the outlet is a bang-bang oscillator (hand
simulation). Demo `scenarios/furnace_outlet_control.toml`, `K = 0.015`,
`T_i = 10 s`; halving the flow reaches the bound.

**What M19.1 found building it.** (i) **All twenty-two pre-M19 plants are
byte-identical on both fidelities with no iteration count moved**; "runs
byte-identical" means post-M19.1 identical, unchanged. (ii) **The hand simulation
held on the engine**: the demo is within 0.06 K of 60 °C from tick 154, the M18
tuning moved to the outlet rides a clamp on 393 of 400 ticks, and the bound falls
between `K = 0.030` and `0.031`. (iii) **The stagnant-outlet hazard is the STARTUP
placeholder, not a mid-run stall**: a furnace that has flowed holds its last
outlet, which a settled loop reads as zero error, while one shut from load reads
20 °C ambient and would wind a PI loop onto full firing by tick 14. Gate 6 was
rebuilt around that case. (iv) **An outlet loop does not settle to the last bit**:
the flow drifts with the tank's level and a PI loop follows a ramp with a lag of
`T_i/(K·G·dt)` = 20 ticks, so gate 4's tolerance is derived from that
(1.52e-5 K predicted, 1.66e-5 K seen), not chosen. All eleven mutations are
caught (the note was exactly right on three); gate 1 is blind to a blind tick
that updates against the setpoint, because a pending memory seeded at zero error
writes the position already held. `scenarios/` holds **twenty-three** files.

**M18 is CLOSED (2026-09-23): reverse action — a furnace holding a temperature,
`docs/DEFERRED.md` row E7 for the duty actuators.** Taken on a DECISION (the
user's, on gameplay grounds, as M17); the demo fires E7's own trigger by being
built. Two slices: **M18.0** (DESIGN §22, six forks, eight gates, ten mutations,
no code) and **M18.1**, which built it — read §22's "Corrections from building
it". Reverse action on a valve is new row **E8**; the next milestone is chosen from
`docs/DEFERRED.md`.

**What a reverse loop is.** A loop declares `action = "direct" | "reverse"`
(absent = direct, a true statement about every pre-M18 loop). The sign has ONE
owner, `ControlledValue::error(measurement, setpoint, action)`, and three callers
that must all pass the loop's own action: `PiController::new`'s load-time seed,
the MANUAL→AUTO seed in `Engine::apply`, and the tick's `update` — a seed taken
against the other sign steps the first output by `2·K·e`. `ControlSnapshot::action`
publishes it, skipped when direct. The declaration is CHECKED against the
actuator: a cooler must be direct, a furnace must say reverse (a furnace loop
with the key absent is refused, not defaulted), and reverse on a valve is refused
(E8: a valve's sign is topology, which the loader does not check). A negative
gain stays refused, now as a second, uncheckable way to say "reverse". The furnace
joined `PlantGraph::actuator_position`/`set_actuator_position` with the cooler's
map (`duty / max_duty`), and `SetFurnaceDuty` gained the cooler's two guards
through one shared function, `Engine::check_loop_owned_duty`.

**Four things the next milestone inherits.** (i) **All twenty-one pre-M18 plants
are byte-identical on both fidelities with no iteration count moved**; "runs
byte-identical" means post-M18.1 identical, unchanged. (ii) **A release gate
that steps all the way back to the old setpoint cannot see a sign**: from the
ceiling, a right and a wrong memory both clamp to zero. The shipped anti-windup
gate steps back to just below the tank and asserts the released output by hand.
(iii) **The inverted `(1 − u)·max` map REGULATES** — mutation 10 settled within
9 kW of a hand-written duty — and only the faceplate gives it away, which is fork
1's reason for refusing it, measured. (iv) "Pinned at the clamp" is not "exactly
1 on every tick": near a ceiling the output dips a hair off and back as the error
shrinks, so a gate that reads one tick can land either side. All ten mutations
were caught. `scenarios/` holds **twenty-two** files; four declare
`[[controls]]`, and `tank_temperature_heating.toml` is `tank_temperature_control.toml`
mirrored — diff them to see what reverse action costs.

**M17 is CLOSED (2026-09-23): the third controlled variable, temperature —
`docs/DEFERRED.md` row E1b's temperature half.** Two slices, M17.0 and M17.1. The
furnace loop (E7), the zero-volume measurement and flow control stay in the
ledger; no row was re-measured, so the next milestone is chosen from
`docs/DEFERRED.md`. Taken on a DECISION (the user's,
on gameplay grounds); nothing was past its trigger. **M17.0 landed 2026-09-23** —
the design note, DESIGN §21, seven forks, eight gates, eight mutations, no code.
**M17.1 landed 2026-09-23** and built it — see the M17.1 paragraph below, and
§21's "Corrections from building it". Four things M17.0 found first.

**The deferral's reason was false — the fourth recurrence of stored-versus-solved.**
"A temperature really is absent before the first tick" (said below in the M10 box,
in E1b, in two `core::graph` docs and in a user-facing refusal in
`build_controls`) is false for holdups: `TankState::temperature` and
`VesselState::temperature` are on the graph, exact from load. Measured: the
snapshot's `temperature_k` at tick `n` equals the graph's at `n − 1` bit for bit.
True only of zero-volume nodes (a furnace/cooler outlet), where it stays deferred.

**The first loop is a COOLER holding a TANK, not a furnace outlet.** A furnace
actuating a temperature is reverse acting (E7), and an outlet temperature is a
zero-volume measurement — the textbook loop needs both missing pieces. The
direction rule is reworded a third time: "raising the output must lower the
measurement", which an inlet cooler satisfies.

**The actuator is the new machinery**: eight valve-only sites (grepped; a first count said six), a `max_duty_mw` on
the LOOP (not the cooler — `NodeSnapshot::kind` would move every cooler plant),
an AUTO guard and a range refusal on `SetCoolerDuty`. **Unit trap, mirrored from
M10**: `setpoint_c` takes `+273.15`, `gain_per_k` takes NOTHING (a °C difference
is a K difference, `smearing_k`'s precedent). Demo: `tank_temperature_control.toml`,
hot water → cooler → tank → drain, 60 °C at ~60% of a 2 MW cooler.

**What M17.1 built.** `variable = "temperature"` on a `tank` or `vessel`, with
`setpoint_c` (+273.15), `gain_per_k` (converted by nothing) and `max_duty_mw` (on
the loop; required on a cooler, refused on a valve). The actuator side has ONE
owner, `PlantGraph::actuator_position` / `set_actuator_position`: a valve's
opening read bare, a cooler's `duty / max_duty`. The loader's seed, passes 1 and 3
and the MANUAL→AUTO transfer all go through it. `SetCoolerDuty` is refused under a
loop in AUTO, and above the loop's range in either mode. The pairing table
refuses every other actuator per variable with its own reason, including the
furnace, which points at E7. **All twenty pre-M17 plants are byte-identical on
both fidelities with no iteration count moved**, the two loop plants included.
The demo declares 0.5 MW (u = 0.25), not the 1.2 MW the note sized, because at
1.2 MW the MANUAL twin parks at 60.04 °C and no gate could tell it from holding.
It parks at 71.71 °C instead. The shipped tuning never clamps; the anti-windup arm
is reached by a 45 °C setpoint command and by the vessel fixture's own startup.
All nine mutations are caught. The one the note predicted the demo would miss
(MANUAL tracking the raw duty) is caught by the demo's own MANUAL twin, and missed
by the transfer fixture the note named.
`scenarios/` holds **twenty-one** files.

**M16 is CLOSED (2026-09-08), and its scope was `docs/DEFERRED.md` row B15 —
one constant heat capacity per cut.** Three slices: **M16.0** (DESIGN §18) a
scoping PROBE barred from committing forks, **M16.1** (DESIGN §19) the design
note that commits them, and **M16.2** (DESIGN §20) the building slice — the
first in six that is not a decision on M11’s licence, because B15 was genuinely
past its trigger. **B15 is NARROWED, not struck**: the gas clause ships, the
liquid clause is untouched because the citation for a liquid `cp(T)` does not
exist in usable form. **Read §20 first; it corrects §19, which corrects §18.**

**What a shaped plant now is.** `EnthalpyModel` is a trait in `core::traits` with
eleven methods — the specific enthalpy, the flux, the stock, the mean capacity
over an interval, the spot capacity, the specific internal energy, both
inversions, the mix, and a provided `stream_enthalpy_flux` that keeps M13’s
`ṁ·(h + λ)` shape. `solvers` holds `ConstantEnthalpy` (today’s arithmetic,
verbatim) and `LinearCpEnthalpy` (`cp(T) = c₀ + s·(T − T_REF)`, so
`h(T) = c₀·y + ½·s·y²`). `[fidelity] heat_capacity` selects, defaulting to
`"constant"`. **The four free functions in `core::energy` that expressed the
datum are gone** — they are methods on the seam now, and `&dyn EnthalpyModel` is
threaded through nine call sites plus `BoilOffModel::boil_off`. **No root find
crossed into `core`**: the shape inverts in closed form, which is why fork 2
admits only shapes that do.

**Ten things the next milestone inherits.**

**Changing the ASSOCIATION of a product is not a refactor.** A probe that
regrouped `(ṁ·cp)·(T − T_REF)` to `ṁ·(cp·(T − T_REF))` — no seam, no shape,
nothing else — moved **11 of the 19 plants on each fidelity, and not the same
11**. So `ConstantEnthalpy` holds the pre-M16 groupings verbatim rather than
composing them out of `specific_enthalpy`, with a unit gate asserting the two
forms agree to a few ULP so the override cannot hide a real disagreement. **All
nineteen pre-M16 plants are byte-identical on BOTH fidelities with no iteration
count moved**; from here "runs byte-identical" means post-M16.2 identical.
`scenarios/` holds **twenty** files.

**A declared constant nothing reads is the failure mode this project refuses, and
the note shipped one.** §19 left `cp_j_per_kg_k` in place alongside a shape. It
is now `Option`, **required** under `"constant"` and **refused** under
`"linear"` — `density_kg_per_m3`’s rule one field up — and
`PseudoComponent::cp` is derived from the shape at the enthalpy datum.

**`cp` has three consumers and one of them has no shipped caller — measured,
not enumerated.** `mean_cp` is read at exactly one site (the boil-off flash) and
no shaped plant boils, so `LinearCpEnthalpy::mean_cp` is gate-only today; the
mutation that replaces it with a spot value is corpus-inert on both fidelities.
Ledger row **B24**. The pipe’s outlet temperature and the exchanger’s `C_min`
take the **spot** value, which is the answer §19 asked this slice to record.

**"A single-inflow chain never mixes" is FALSE.** `mix_inflows` runs for a holdup
with one inflow too, and there the constant model’s capacity-weighted average and
the shaped model’s `T(h)` are different numbers the moment `cp` is not flat.
Substituting one for the other moves the demo’s bytes and **diverges the game
fidelity’s solver at tick 170**.

**The citation is METHANE, because that is what the corpus declares.** NIST
WebBook CAS 74-82-8, Shomate from Chase (1998), NIST-JANAF 4th ed. The file
declares a linear fit over the demo’s own 300–800 K span; its worst residual
against the source is **1.7346%** and gate 2’s band is **1.4× that, computed
in-test rather than chosen**. The counterfactual is the flat 2 220 J/(kg·K) the
five existing gas plants declare, which misses by more than 40% at the top of the
range.

**The demo moves a published number by 304 K.** `scenarios/fired_gas_drum.toml`
— methane header → 0.56 MW fired heater → 4 m³ surge drum → sink — settles at
**802.438 K** shaped against **1 106.863 K** constant, inventory 6.559893 kg
against 4.822575 kg, worst relative movement **0.548 at tick 140**. Its shape came
straight off §19’s finding that the two gas plants that moved are exactly the two
carrying a `Vessel`: **the mechanism is a holdup that integrates a temperature
over a span, not gas-ness.** **The M7/M12/M14/M15 pair pattern is departed
from** — the refusals make the twins differ in four lines rather than one, so the
constant twin is derived in-test and the contrast is not runnable from the CLI
(row **B25**).

**The milestone’s own boundary is REFUSED at load rather than shipped quietly.**
The cascade’s two duties and the compressible valve’s `γ = cp/cv` read a
constant capacity from inside solver traits whose signatures this slice did not
change, so a shaped plant paired with either would run half its arithmetic on a
constant. Both pairings are refusals that name themselves (rows **B22**, **B23**).
So are: a shape with no `[[components]]` block, a partly-shaped slate, and a
component with no shape at all.

**Three gates had no power over their own subjects, and each is a rule.** (i)
**Gate 1 was GREEN under the mutation it was written for**: it measured a
difference of two equal-width intervals, and for a linear shape the mean over
`[a, b]` is the spot value at the MIDPOINT — so reading it at the left edge
shifts both answers equally and the difference is unmoved. **A gate built on a
difference is blind to a constant offset in its own argument.** (ii) The
discarded-root gate computed both roots by hand and never called the model, so it
defended arithmetic rather than the shipped branch. (iii) The refusal sweep had no
case for the refusal the mutation deletes — its partial-shape case exercises a
different refusal. All three are strengthened and all three now fire.

**Every mutation prediction that named an EXISTING reference test was
structurally impossible, for one reason: all nineteen pre-M16 plants select the
constant model**, so a mutation of the shaped one cannot reach any test written
before this milestone. That is also why the demo’s own two tests did most of the
catching. **Two of §19’s six mutations are not expressible** — "stop the inverter
after one step" has no subject (closed form), and "give the constant model a
`mean_cp` that ignores its interval" IS the shipped behaviour — and the
substitutes are stated rather than quietly swapped.

**A refusal nobody mutated is a refusal nobody measured, and both boundary
refusals were passing on the wrong half of a disjunction.** The cascade case
asserted `separation = \"cascade\"` (with backslashes the error does not carry)
OR the bare word "cascade" — so the bare word was doing all the work, and any
message mentioning a cascade would have passed, including the cascade loader's
own refusals. Both assertions now name a distinctive substring of their own
refusal's message, and deleting either refusal fires the sweep at
`panic!("this plant should not have loaded")`, which is the right reason.
**The boundary is exactly two sites and that was audited, not asserted**: the
only live readers of `mixture_cp`/`mixture_cv`/`.cp` outside the enthalpy module
are the cascade's two duty terms and the choked-flow `γ`, and both are behind
the refusals. Under a shape the stored `cp` is the shape's value at the DATUM —
the coldest value it takes — so a missed site would read the coldest capacity as
if it held everywhere.

**A bytes claim gets a gate; a cost claim does not.** The substitute for mutation
5 — giving `ConstantEnthalpy::mean_cp` the difference-quotient form — failed
**zero** tests and moved **three** boiling plants on both fidelities. M9.3b left
an uncaught mutation because a gate for it would assert a *cost*; this one
asserts *bytes*, which M8.5 and M13.1 both closed, and it was defended by nothing
but a baseline file on the measurer’s disk — CI commits no baseline. It is now
asserted bit-equal in the grouping gate, **over deliberately non-round spans**:
the first draft was green under the mutation because `(h(t₂) − h(t₁))/(t₂ − t₁)`
reproduces a flat 2 220 J/(kg·K) exactly over 300–700 K, while the engine’s
arguments are a bubble point and a tank temperature and roughly two in five such
spans differ by an ULP.

**Two instrument findings.** A catch set taken under `cargo test`’s default
fail-fast is a **LOWER BOUND**, and binary ordering decides which subset is
visible — one mutation read as two failures and is six. Use `--no-fail-fast`. And
the harness reported `corpus exit=1, 0 moved rows` three times while the corpus
binary had never executed (a forward-slash relative path handed to `cmd.exe`),
which reads exactly like a real result: **an exit code with no rows behind it is
not a result.** Same shape as M9.1’s first probe.

**A3 is unmoved and is the nearest row that has not fired**: 920 of 5 000 sweeps
on `relief_blowdown`. The new plant’s own worst is 9 on the Newton fidelity and 7
on the game one.

**M15 is CLOSED (2026-09-08), and its scope was THE SECOND RECOVERY STAGE —
`docs/DEFERRED.md` rows B18 (a vent chain longer than one hop) and B17 (a
condenser's `UA` in a field documented as insulation).** Nothing was past its
trigger, so this is a DECISION on M11's licence, the fourth; and **both rows are
triggers the milestone fired by building its own demo**, which is M13's shape.
**M15.0 landed 2026-09-07** — the design note (DESIGN §17, five forks, seven
gates, six mutations, no code). **M15.1 landed 2026-09-08** and built it:
`scenarios/crude_column_recovery_train.toml`, the fork-4 rename, seven gates and
a ten-edit mutation pass. B17 and B18 are both struck.

**A chain needed NO CODE, and that was confirmed on the shipped tree before
anything was edited.** `core`, `solvers` and the loader are untouched by the
train; the only source changes in the milestone belong to the rename. Over
6 000 ticks the train takes recovery from **42.431605%** to **85.620291%**, on an
emitting half that vents **6 709.6978 kg** either way. `scenarios/` holds
**nineteen** files and the new one is the fifth member of the pair-diff family.

**Six things the next milestone inherits.**

**The sort was ALREADY defended, so B18's "built and unused" was wrong about the
gate as well as about the mechanism.** Making `holdup_evaluation_order` return
`node_ids()` unconditionally is caught by **M14.1's own
`moving_the_drum_up_the_file_changes_no_number`**, which builds a reordered
depth-1 graph in-test. Gate 5 stays green, so the sort really is inert on the
shipped train — M15.0's finding (i) survives. **What depth 2 bought is the
TRANSITIVITY assertion, not the first exercise of the machinery**, and gate 3
asserts it by enumerating all 36 transpositions of the reordered plant's node
order and finding none that satisfies the constraint set, with the depth-1 plant
reordered the same way beside it as the control, where one swap suffices.

**A rename has a blast radius in TWO directions and B17 named neither.**
`NodeSnapshot::kind` serializes `TankState`, so `ambient_ua` is a PUBLISHED name
on every tank of every plant — the thorough rename would have moved eighteen
plants' bytes for a spelling. And `NodeDef`/`PipeDef` carry no
`deny_unknown_fields` (only `ControlDef` does), so the minimal rename would have
let an older file parse, drop the key and run with `UA = 0` —
`crude_column_recovery.toml` recovering 20.7% instead of 42.4% with nothing
saying why. **What ships stops at the LOADER**: `ambient_exchange_ua_w_per_k` is
the TOML key on both tank and pipe, `graph.rs`'s two Rust fields keep their
spelling, and the old spelling ships as a tombstone refused by name with the
counterfactual as a test. Gate 7 asserts both ends on the serialized bytes.

**The note's own mutations were not expressible at the site the note pointed
at.** The receiver block in `engine.rs` is ONE site shared by both hops, so
dropping the arriving latent term there breaks hop 1 too and is caught by every
hop-1 gate M14.1 already shipped — §17 predicted "gate 5 blind" for a second-hop
drop and gate 5 fired, for the wrong reason. **The first attempt to scope them
was VOID and read as an escape**: the scope test asked whether the emitter vents
at all, which is true of every tank on every boiling plant, so both edits ran
fifty-six binaries and changed nothing — and the first write-up called 2b
uncaught and gate 6 blind to the second hop. **An inert edit and an uncaught edit
are the same observation.** The test that works is one step up — the emitting
tank is itself receiving somebody else's vent — and run that way 2b fires gate 6
and gate 1 alone (residual 1.5435e-13 → 2.1754e-2, a 1.12e9 J hole equal to
hop 2's own latent arrival) and 3b fires gate 2 alone, as §17 predicted.
**Read a catch set for WHY each catch fired before counting it**, which is also
what mutation 1 needs (its early return skips cycle detection, so four of its six
catches are measuring mutation 4 instead).

**An uncooled second stage still fractionates.** Setting stage 2's `UA` to zero
fires gate 1 — recovery falls **85.620291% → 51.954037%** against one stage's
42.431605% — and leaves **gate 2 green**, because stage 2's separation comes from
stage 1's vapour already being light, not from stage 2's own cooling. The
condenser decides how much is retained; the chain decides what. §17 predicted the
compositions would converge and they do not.

**Two of M15.0's own figures were wrong and both were understatements.** Stage 2's
FEED switches on at tick **2 752** and stage 2 itself does not vent until
**3 834** — 64% of the shipped run, not the note's "about half". And the emitting
tanks' indifference to where their vapour goes is **2.7e-9 / 3.7e-9** relative
one hop further out, against M14.1's 8.7e-11 — one more node, one more order of
the same deliberately-unpinned mechanism — so gate 1's bound is `1e-8`.

**From here, "runs byte-identical" means post-M15.1 identical, which is
unchanged: all eighteen pre-M15 plants are byte-identical on BOTH fidelities**
with no iteration count moved, against a baseline recorded before the first edit.
The three rows M15 leaves behind are **B19** (a coolant colder than ambient),
**B20** (a chain deeper than two) and **B21** (a condensate that rejoins the
process), and all three cost "a scenario file or a slate" — which B18's own
correction now says is a weaker argument than it sounds.

**M14 is CLOSED (2026-09-07), and its scope was THE RECOVERED VAPOUR —
`docs/DEFERRED.md` rows B12 (condensation) and B13 (where the vented vapour
goes), taken together because they cannot be split.** M14.0 wrote the note
(DESIGN §16, seven forks, six gates, nine named mutations, no code); **M14.1
landed 2026-09-07** and built it — a per-tank `vent_to` key, the receiver side of
a boil-off vent, `PlantGraph::holdup_evaluation_order`,
`scenarios/crude_column_recovery.toml`, nine gates and two hand-built fixtures.
B12 is struck; **B13 is struck for RECOVERY only** — a flare is combustion (B7)
and fork 5 refuses a `Sink` destination, so its emissions clause stays open.

**What a recovery drum now does.** A tank's boil-off vent can name another tank.
The receiving holdup counts the vent as an inflow, reads the stream WHOLE — flow,
composition `y = K·x`, temperature and `latent` — and books it through
`energy::stream_enthalpy_flux`, so the arriving vapour's `c̄p·(T − T_REF) + λ` is
credited rather than its sensible half. It condenses **entirely**; whatever
cannot stay boils back off through the drum's own vent at the drum's own
equilibrium, so partial condensation is the composition of two terms that already
existed and **no stream is ever part liquid and part vapour** (which is what
keeps this out of B3). The condenser is `ambient_ua_w_per_k` — `Q = UA·(T_AMB −
T_B)`, self-limiting, in a field documented as insulation, which is a new ledger
row rather than a thing fixed here.

**There is no seam, and fork 6 was right.** `traits.rs` is untouched, there is no
`[fidelity] condensation` key, and `schema.rs` gained exactly one optional field.

**Nine things the next milestone inherits.**

**All seventeen existing plants are byte-identical on BOTH fidelities with no
iteration count moved** — the claim M13 could not make for the whole corpus,
measured against a baseline from `HEAD` in a separate worktree. From here, "runs
byte-identical" means post-M14.1 identical, which is unchanged. `scenarios/` held
**eighteen** files at M14.1 and the new one is `crude_column_boiloff.toml` plus a
drum and two keys — the pair pattern extended a fourth time.

**There is a FOURTH receiver-side site and it is the one the note quotes as its
own precedent.** `engine.rs:701`'s vent-finding predicate tests the NODE's kind,
never the edge — loop-invariant — so on a recovery drum it reduces to "I am a
Tank and this is a vent" and returns a SOURCE tank's vent. The drum would publish
its boil-off onto that tank's edge: M12.1's 2 539 kg bug at the site whose
comment records paying for it. **M12.1's `NodeKind::Tank` test was right while
every vent ended at an `Atmosphere` and stops being right the moment both
endpoints are holdups.** Meanwhile `engine.rs:1036`, which the note marks for
work, is correct and stays.

**Ownership is STORED, not derived.** `LeakRole::BoilOffVent { emitter: NodeId }`,
with `boiloff_vent_emitter` the single predicate both sites read in opposite
directions. A vent stored the other way round is now a *correct* graph rather
than a tripwire, so fork 3's promised fixture is a real gate.

**The evaluation order is machinery the note did not name, and gate 1's tolerance
is the argument.** A vent is written at the END of its emitter's iteration and
read at the TOP of its receiver's, so the wrong order parks `ṁ_v·dt` in flight
every tick — 1.69 kg against the run's 6 709.7 kg, 2.5e-4 systematic.
`holdup_evaluation_order` is a stable Kahn sort constrained by **tank → tank
vents only**, so every pre-M14 plant takes an early return and gets `node_ids()`
by construction. **It re-premises fork 5's cycle refusal**: not "a cycle makes
the answer depend on node order" but "no order exists".

**Gate 3 as the note words it is falsified by the CORRECT engine.** The drum
holds 0.2614 light naphtha and `naphtha_tank` holds 0.4971 — the note's
inequality points the wrong way, because the drum takes vapour from two tanks and
the heavier carries 2.3× the flow. The reference that works is the flow-weighted
mix of ALL the emitting liquids (0.1523), against which the drum is 1.72× richer
and 44× leaner in the heaviest cut. **A gate written against one member of a set
can be falsified by the set.**

**Fork 4's argument for the condenser is FALSE in its stated mechanism.** An
uncooled drum does not recover nothing: it recovers **20.72%** and
**self-fractionates into a heavy pot**, boiling the light material back off until
its own bubble point has climbed past 435 K and it has stopped re-venting
entirely. The shipped `UA = 3.5e4 W/K` recovers **42.52%** over the run and holds
**49.87%** at tick 6 000; `1e5` recovers 100%. The knob discriminates, which is
what M7.1's `smearing_k` rule demands — but the case for it is "otherwise half as
much, and what is kept is the bottom of the barrel". **Recovery is also not
monotone in `UA` at the low end** (20.72 → 17.35 → 23.75 → 42.52 → 100).

**Adding an inert node to a plant is not bit-neutral**, and it took a control to
say so: an inert `spare_tank` on `crude_column_boiloff.toml` moves the other
tanks' vented mass by 7.1e-11 and 9.8e-10, the same order as the recovery plant's
own 8.7e-11. The mechanism is deliberately unpinned.

**The `Vessel` refusal needed a plant of its own** — a vessel must hold a gas
composition, refused one pass earlier for the wrong reason with the right exit
code. A refusal is only tested if the thing it refuses could otherwise be built.

**Four of the note's nine mutation predictions were wrong, and the two that were
exactly right were already paid for.** (2), the receiver reading the upwind
node's liquid `x`, is caught by gate 3 and nothing else — M12.1's finding,
confirmed. (5), scoping by incidence direction, is inert on the demo and caught
by the hand-built fixture alone, exactly as fork 3 said. What was wrong: (1) and
(9), dropping the latent term, spread to gates 1 and 4 as well as 2, because the
arriving enthalpy is what decides whether the drum ever boils — the note treated
`λ` as bookkeeping and it is a state variable's forcing. (4), "the skip scoped by
node kind alone", was predicted **inert** and is identical to (3): on this plant
the RECEIVER is a Tank, so scoping by kind makes the drum skip its own inflow. It
is inert only on a plant whose vents all end at an `Atmosphere`, which is the
whole corpus before M14 and none of it after. And (8), removing the cycle refusal
from the loader, fires the scenario gate and NOT the fixture — the two callers of
one function are independently defended, which is the right answer.

**Gate 5 came for free, both trees in one probe, as fork 5 predicted it would.**
Scaling `BoilOff::latent_heat` by 3.7× at the construction site only — M13.1's
own write-only probe — over 3 000 ticks: the emitting `naphtha_tank`'s
temperature, mass and vent rate are **bit-identical**, and the receiving drum's
mass falls 4 019.4 → 694.8 kg, its temperature rises 357.6 → 408.5 K, its light
fraction falls 0.811 → 0.112 and its own vent nearly doubles. The same edit that
moved nothing before this milestone now moves only the receiver.

**Gate 2's tolerance was WRITTEN before it was measured, and four numbers were
wrong.** The first draft said the residual "rises like `1/dt`", quoted 8.0e-13
for it, and put the arriving latent share at 45.09% of 4.10e9 J — none of it run.
Measured: the residual is **8.5104e-14**, the drum receives **4.121041e9 J** of
which **1.751830e9 J (42.51%)** is latent (reproducing M14.0's own probe), and
mutation (1) misses by **3.35e-2**, twelve orders above the bound. **The `dt`
discriminator also gives a weaker answer than it did for M13.1**: 8.5104e-14 →
6.3446e-14 → 2.4341e-13 at `dt`, `dt/2`, `dt/4` — not monotone, so neither
`1/dt` nor truncation. What survives is the inference the tolerance needs, that
the bound is not sized against an engine error, and the shipped assertion is "it
does not fall like a truncation term" rather than "it rises".

**The evaluation order's per-tick cost is argued structurally and NOT measured,
because nothing available can measure it.** The line it replaced was already a
`node_ids().collect()`; what is added is one pass over `edge_ids()` and a
`BTreeMap` that stays empty on the seventeen plants whose vents all end at an
`Atmosphere`, where the function takes its early return. The corpus wall-time
column cannot settle it — `crude_column_boiloff` reads 199.4 ms before and
305.0 ms after on a plant proved byte-identical, which is machine drift under
this project's own rule.

**The mutation harness corrupted the tree, for the reason already on record.**
Two accidental concurrent runs; five mutations in two crates reported the
identical energy residual to seven figures, which is arithmetically impossible
and is the signature of a reinstated edit. The shipped pass holds a lock and
verifies every anchor afterwards.

**M13 is CLOSED (2026-09-07), and its scope was the LATENT HEAT of a boil-off —
`docs/DEFERRED.md` row B16.** M12.1 left the vent carrying its vapour's sensible
enthalpy and nothing else, so `m_v·Δh̄_vap` appeared nowhere and every flashing
plant's energy books showed a sink. **M13.0** wrote the design note (DESIGN §15,
seven forks, four gates, eight mutations, no code) and **M13.1 landed 2026-09-07**
and built it: `Stream::latent: Option<JPerKg>`, `BoilOff::latent_heat`,
`energy::stream_enthalpy_flux`, the datum correction, four gates, a `dh_vap`
reference envelope, and **I6b — the first energy invariant in this workspace ever
evaluated on a plant that boils.**

**Nothing selected this milestone: it is M11's licence (a decision), and it fired
its own trigger.** B16's trigger is "a consumer of a flashing plant's external
energy balance" and it names three; M13's first artefact is one of them. That is
written into the note's first paragraph rather than left to be noticed.

**A stream is now self-describing for energy.** Its specific enthalpy is
`cp·(T − T_REF) + latent.unwrap_or(0)`, and `energy::stream_enthalpy_flux` is the
single owner of that expression. `latent` is `None` on a liquid — not `Some(0)`,
which would say "a vapour whose latent heat is nothing" — and
`skip_serializing_if` is what keeps sixteen plants byte-identical. **It is not
B3**: one scalar on a single-phase vapour stream, not a stream that is part
liquid and part vapour. **`is_boiloff_vent()` left the energy path** as the note
predicted, so B12's condenser and B13's flare need no new discriminator.

**The datum sentence was the thing that was actually wrong.** `h = cp·(T − T_REF)`
is the datum **for a LIQUID** — the reference state is saturated liquid at
`T_REF` — and §4a, `energy.rs` and `Stream` all now say so. A cut the slate
*declares* `Gas` is untouched: it never condenses in this engine, so its own
datum is self-consistent and `capacitive_vessel_reference.rs` is right as it
stands.

**Six things the next milestone inherits.**

**The number this milestone was licensed by counted ONE of the demo's TWO boiling
tanks.** M12.1's 7.6084e8 J and 1.54 MW are the naphtha tank alone — reproduced
here to five digits once the sum is restricted to that vent. The plant's real
hole is **1.751826e9 J over 6 000 ticks and 4.317708e6 W at tick 6 000**, because
`distillate_tank` boils harder (11.73 kg/s of vapour against 5.17) and nothing
had counted it: **2.30× and 2.80×**. M12.1's own prose named two boiling tanks one
sentence from the figure. **A measured number can be right about its subject and
silent about its scope.**

**"All seventeen plants byte-identical" is FALSE as stated and true in what it
meant, and the corpus cannot tell the difference.** Sixteen are identical on both
fidelities; `crude_column_boiloff` moved and *had to* — the fingerprint is taken
over published snapshots and the point of the milestone is to publish a field
there. What replaces the claim is the M8.5 method: strip the `latent` keys from
the after-file and compare. **843 keys per fidelity, and both after-files
reproduce their before-file byte for byte.** From here, "runs byte-identical"
means post-M13.1 identical.

**The note's load-bearing prediction was RIGHT, and the probe it named is what
proved it.** Scaling the reported `latent_heat` by 3.7× *at the construction site
only* — so the flash fraction keeps its unscaled divisor — moves nothing but
`latent` on either fidelity. Perturbing `dh_vap` instead would have moved the
whole plant and could not have answered the question.

**Gate 1's tolerance is float noise, and the discriminator is which way it moves
under `dt`.** 4.58e-12 at the shipped timestep, 7.0e-12 at half, 1.40e-11 at a
quarter — it **rises**, like `1/dt`, where truncation would fall. The source is
the gate's own `ΔU`, a difference of two ~1e10 J inventories, not the engine.
Bound `1e-9`, eleven orders below what the same sum reports without the term.
**And gate 1's power comes entirely from the TANKS**: the cascade's reboiler duty
is defined as its condenser duty plus the column's own external balance, so the
column's contribution closes by construction and audits nothing.

**Gate 4 as the note specified it could not have caught its own mutation.** The
note put specific-versus-total at "a factor of `m_v` — order 10³"; the vents move
**0.517 kg and 1.173 kg per tick**, so a total sits 0.5–1.2× the specific value
and no magnitude band sees it. Gate 1 catches it, through the `mass_flow`
`stream_enthalpy_flux` multiplies by. What ships is a mixture-average bound and
an intensivity test under a halved timestep.

**The one mutation §15 said gate 1 would catch is UNCAUGHT, and the reason
generalises.** A stale `Some(λ)` left on an idle vent is invisible to any energy
balance, because the term is SPECIFIC and an idle vent's `mass_flow` is zero —
the very property that made "specific rather than total" the right shape (one
formula, both branches) is what stops a balance policing the field. And the demo
cannot reach the state: **no vent on it ever goes from boiling back to idle**
(naphtha publishes `latent` from tick 1 210, distillate from 2 380, bottoms
never — both figures at the run's 10-tick snapshot resolution, and the naphtha
one reconciling with the file's 1 825 through M12.1's 618-tick blanket-pressure
correction). Closed with a fixture, not a gate on the demo — a hot holdup with no
inflow, which boils once and is idle for the next forty-nine ticks, asserting
`carrying vapour ⇔ latent.is_some()` with both arms shown reached.

**`dh_vap` is anchored for the first time, on the SLOPE of a citation already in
the repo.** `reference/vapour_pressure.rs` carries NIST's Antoine coefficients
for n-hexane; Clausius–Clapeyron turns that fit's slope into **30 515.9 J/mol**
against Trouton's **30 086.4**, a ratio of **0.9859** inside a `[0.85, 1.15]`
band sized from two one-sided errors (the ideal-gas assumption biases the anchor
high; Trouton is a ±10% correlation). Without it gate 1 is a consistency check
wearing a physics label — halve `dh_vap` and the flash boils twice the mass at
half the latent heat and every book closes to the last digit (verified by
searching for gate 1's name, because the claim is that it PASSES). **But the
note's "the workspace has no reference test for `dh_vap`" is false as written**:
two `thermo::tests` already pin it. Both compare the model **against itself** —
one against the workspace's own constant, one against its own vapour pressure —
and neither can see a wrong `TROUTON_CONSTANT`. Gate 3 is the only test
comparing a latent heat's magnitude to data from outside the workspace.

**M12 is CLOSED (2026-09-07), and its scope was the two-phase HOLDUP — a quarter
of `docs/DEFERRED.md` B3.** It is the first milestone opened because the ledger
said a hurdle had arrived: B3 was the only row on the wrong side of its own
number. **M12.0** wrote the design note (DESIGN §14, nine forks, seven gates,
eleven mutations, no code); **M12.1 landed 2026-09-07** and built it — the
`BoilOffModel` seam with `NoBoilOff` and `FlashBoilOff`, `[fidelity] boiloff`, the
loader-built vent edge, `scenarios/crude_column_boiloff.toml`, ten gates and a
twelve-edit mutation pass. **Nothing in `docs/DEFERRED.md` is past its trigger
again**; B3 itself stays open on its three STREAM paths.

**What a boiling tank now does.** A liquid holdup above its own bubble point
vaporises the superheat, parks on its bubble point, and the vapour leaves at
`y = K·x` through a vent edge to `Atmosphere` that the LOADER builds — one per
tank, plus the atmosphere node, whenever the plant selects a model that boils.
The mass that boils is the mass whose latent heat absorbs the excess enthalpy
(`f = c̄p·ΔT/Δh̄_vap`), an enthalpy constraint rather than a rate law, so there is
no constant to tune.

**Six things the next milestone inherits.**

**The note's headline number was measured at the tank's DECLARED composition, and
the note names that trap one section earlier.** `f_in = 0.236` rests on
`T_bub = 354.3 K`, which is essentially pure light naphtha's normal boiling point
— the tick-0 declaration. Against the mixture the tank actually holds
(`T_bub = 369.77 K`) the prediction is 0.1282 and the plant delivers **0.1090**;
over the run **8.93%** of the naphtha draw. So a tenth boils off, not a quarter.
**A number in a design note is a hypothesis with formatting**, and this one was
wrong in the exact way its own text warned about.

**The defect that mattered was in the ACCOUNTED PATH, not the physics: the
atmosphere is a node too.** The tank's inventory fell by 2 539 kg while its vent
reported 0.0 kg/s on every tick — step 3 loops over every node, the atmosphere
has no holdup, and its iteration wrote a zero over a rate a tank had published.
Caught by a gate's CONTROL, not by any assertion about vapour, and by no
conservation test (I1 is not evaluated on a plant that boils). **A shared edge
belongs to one of its endpoints, and a per-node loop must say which.**

**The gate the note called "the only one that separates a flash from a decrement"
did not.** Removing mass at the tank's own composition still removes mass, so the
tank blends its inflow differently and its composition parts company with its
non-boiling twin's either way — the two-plant comparison stayed green under the
decrement. What discriminates is the VENT's own published composition against the
holdup's: a vent carrying `x` is not richer in the light cut than `x`. **A gate
comparing two plants measures two trajectories; a gate comparing two streams at
one instant measures the thing itself.**

**The latent heat ships with no accounted path, and it is not small.** The holdup
datum is `h = cp·(T − T_REF)` — sensible only — so the vent carries the vapour's
sensible enthalpy and nothing else. **7.6084e8 J over 6 000 ticks, 13.9% of the
enthalpy the draw delivered, 1.54 MW at tick 6 000.** Mass balances exactly; the
energy books show a sink at every boiling tank. Ledger row **B16**, **closed by
M13.1** — where those figures turned out to be the naphtha tank alone and the
plant's own are 1.751826e9 J and 4.317708e6 W. The fix that
suggests itself — a vent temperature chosen so the sensible enthalpy comes out
right — is a fabricated number and is refused.

**Two specification corrections found by building.** The boil-off is evaluated at
`P_ATM`, the tank's BLANKET pressure, not its node (floor) pressure — the floor is
35 kPa higher on the demo, would make the term a function of LEVEL, and moves the
first boil by 618 ticks. And the clamp at `f = 1` is **provably subsumed** by a
per-component cap the note never specified: the vapour is enriched, so an enriched
component runs out before the total does, and `min_c(w_c/y_c) ≤ 1` always.

**From here, "runs byte-identical" means post-M12.1 identical, which is
unchanged: all sixteen pre-M12 plants are byte-identical on both fidelities**,
measured against a baseline recorded from `HEAD` in a separate worktree, with no
iteration count moved. `scenarios/` now holds **seventeen** files, and the new one
is `crude_column_cascade.toml` with one key changed — the M7 pair pattern extended
to a third member. It costs **+17.5% of that plant's own tick** (paired, twice, in
one session), which is 0.06% of a 60 Hz frame.

**M11 is CLOSED (2026-09-06), and its scope was the cavitation criterion.**
M11.0 wrote the note (DESIGN §13, seven forks) and M11.1 built it: a third method
on `ThermoModel`, a per-tick criterion in `Engine::tick`, `NodeSnapshot::cavitation`,
`scenarios/cavitating_pump.toml` and twelve gates. It is **the first milestone
taken on a decision rather than on a measurement** — neither of B1's trigger
clauses had fired, and clause (b), "a frontend needs to display cavitation", is a
decision that was made rather than an event that arrived. What licensed it: §3
told frontends for ten milestones to read a *wrong* number as the signal, and the
M10 close-out's correction left them with nothing.

**The row's own noun was wrong and the milestone ships neither of the things it
names.** B1 says "floor", §3 promised a "clamp"; §13 fork 1 rejects the clamp
because the vapour it would account for is mass in a phase the state vector lacks
— which *is* `docs/DEFERRED.md` B3. M11 ships a **criterion and a signal**, the
hydraulics are untouched, and a cavitating pump still delivers full head. That
disagreement is deliberate and is now its own ledger row (B9).

**The finding that re-premised the row: B1's 1.865× was a number no engine
configuration could produce.** `heat_recovery` declares `thermo = "constant"`,
whose `k_value` is an `Err`; the number came from the M10 close-out's standalone
script. **Fourteen of the fifteen pre-M11 plants select `constant`**, so under
B1's original four-kind node list ("pump, valve, junction or exchanger") the
engine-computable set across the whole corpus was **EMPTY** — every node of those
kinds sits on a plant whose model refuses, and the one plant that can answer
(`crude_column_cascade`) has none of them. Its only flow-path node is a
**furnace**, so §13 fork 4 enumerates all fourteen `NodeKind` variants and the
ledger's trigger now names six kinds. **A distance is a property of the engine,
not of the plant.**

Five things the next milestone inherits. **`None` never means "healthy"** — it
means there is no criterion here (wrong node kind, a gas, a model that cannot
answer, or before the first tick), and fourteen plants emit nothing at all; a
frontend must render that as *unknown*. **The error VARIANT is load-bearing**:
`SimError::Scenario` from a thermo model means "this fidelity cannot answer" and
reports nothing, every other variant fails the tick — three of the workspace's six
`ThermoModel` impls are test stubs and each had to be told which it meant. **An
exclusion cannot be gated on the verdict**: an excluded node publishes nothing, so
the only way to show the exclusion is doing work is to evaluate the criterion
independently and find it would have fired — and the plant has to actually be in
the state, which `crude_column_cascade`'s naphtha tank is not until **tick 1 826**.
**The demo carries no holdup at all**, because the obvious shape (a hot rundown
tank feeding a pump) would have shipped a plant sitting in B3's state. And
**`thermo = "trouton"` now changes numbers on a plant with no column** — M11 is
the first consumer that makes the key matter there, which was measured before it
was relied on (the fidelity switch alone moves nothing: 175 197.988 Pa either
way).

**From here, "runs byte-identical" means post-M11 identical.** Fourteen of the
fifteen pre-M11 plants are unchanged on both fidelities; `crude_column_cascade`
carries exactly one extra key on exactly one node (`preheater`), verified by
stripping the key and reproducing the before-file byte for byte. No solver
iteration count moved. `scenarios/` held **sixteen** files at M11 and holds
**seventeen** from M12.1.

**M10 is CLOSED (2026-09-06), and its scope was the second controlled variable.**
Pressure is built (M10.0 + M10.1) and pressure is all it built — temperature and
flow were explicitly not committed to and stay deferred as ledger row E1b. It is
the first milestone chosen from `docs/DEFERRED.md` rather than handed over as a
defect. E1 — pressure control — was the row; the milestone was scoped one step
wider on purpose, because everything the note argues is machinery the *second*
variable pays for and the third and fourth inherit. **That argument was tested
and came back stronger than stated, so a third variable has nothing left to prove
about the seam**, which is why the milestone closes here rather than continuing.

**The asymmetry between the two variables left behind is recorded, not acted on.**
A temperature really is absent before the first tick — resolved by the tick, not
stored on the graph — so a temperature loop *would* owe the tick-0 rule pressure
turned out not to owe. A flow lives on an EDGE, and `measure` takes a node, so
flow control is a signature change rather than a new match arm. **Neither has a
plant asking**, which is the sentence that keeps twenty other ledger rows
deferred.

**The close-out's own work was `docs/DEFERRED.md` row B1 — the cavitation floor —
and it falsified the sentence the row rested on.** DESIGN §3 told frontends to
read *negative absolute node pressure* as "cavitating". A liquid boils below its
**vapour** pressure, which is positive, so §3's marker fires **late by exactly
that vapour pressure**. Measured on the reference plant with one number changed
(the pump mounted above its tank): cavitation begins at **17.44 m** of suction
lift and §3's marker only fires at **18.02 m** — a band in which the plant is
boiling and reported healthy. It is narrow only because cold water boils at
5 640.6 Pa; on light naphtha at 445.75 K the same marker would be **nine bar**
late. §3's *reachability* claim was true and is now measured: 19 m of lift gives
−9 522.9 Pa with the solver converging and 9.8 kg/s flowing. §3 is corrected:
**the engine does not detect cavitation and has no signal for it**, and a
negative pressure is a symptom after the fact, not a criterion. **The first draft
of this write-up said the marker "can never fire", which was an overstatement
caught by running the probe instead of reasoning about it.** The row's old distance ("the lowest pressure is 100 000 Pa, so nothing
is near a vapour pressure") compared a pressure against **zero** while concluding
something about a vapour pressure it never evaluated — and its 100 000 Pa was a
*declared sink*, not a solved state. Against each node's own bubble point the
tightest solved margin is **1.865×**. B1's trigger is now written: a solved
pressure in the **hydraulic path** (pump, valve, junction, exchanger — not a
holdup) falling below that node's bubble pressure, or a frontend needing to
display cavitation. **The node-kind clause is load-bearing**: without it the
trigger fires immediately on two product tanks, and a tank above its bubble point
is B3's two-phase holdup, not cavitation. **Clause (a)'s reachability was measured,
not assumed** — one edit to one shipped file reaches it — because a trigger no
plant can reach would have been the fifth dead gate in this project's record.

**The sweep moved a different row, and that is the sharper finding.** B3 (phase in
the state vector) said "no shipped plant asks" and guards three paths at load.
There is a **fourth path no refusal names, because it does not exist at load**: a
column draws at real tray temperatures, the product tank has no cooler, and the
tank stores a liquid the model's own correlation says is boiling.
`crude_column`'s `naphtha_tank` sits at **0.30×** its own bubble pressure,
sustained and worsening as the draw heats it (395 K → 432 K over the run), and
reports as liquid throughout. **A load-time refusal cannot catch a state that
emerges during the run.** Whether the fix is a product cooler in two files or
phase in the state vector is left open in the row.

**Three of four categories in that sweep were exclusions, and saying so is part
of the result.** Ten of fifteen plants have a node below its bubble pressure:
five declare gas-phase cuts (a liquid bubble-point test is meaningless there),
two are columns (which are *at* their bubble point by definition), two are the
FCC plants (B3 already). Publishing the headline without the exclusions would
have moved two rows on evidence that does not exist.

**M10.0 landed 2026-09-06** — the design note, DESIGN §12, six forks and five
gates, no code. Four things to know before the building slice.

**The reason §10 gives for deferring pressure is FALSE, and it is the third
recurrence of that exact error.** Fork 3 says a pressure is *solved* and lives in
`last_solution`/`NodeStates`, empty before tick 1, so a pressure loop has no
measurement at tick 0. A `Vessel`'s pressure is `m/C` with `m` on the graph —
**stored**, and exactly the declared figure, because `build.rs` computes the
initial mass as `P · capacitance(slate)` through the same method `pressure`
divides by. The measurement path already exists, `measure`'s signature does not
change, and the promised tick-0 rule is not owed. The claim is true only of a
*junction's* pressure. §3a fork 4 made the same class of error, and §10 fork 3
corrected M8.1's version of it **one paragraph before committing it again** about
the variable it was deferring.

**The ledger's own distance for E1 was wrong: `relief_blowdown` has NO ordinary
valve** — source, vessel, PSV, sink — and the PSV is refused as an actuator by
name. It lacks an actuator, not a variant, and adding one would move a regression
anchor. The demo is a new file, as M8.4's was. Row corrected.

**The schema's prediction `setpoint_pa` is wrong; the keys are `setpoint_bar` and
`gain_per_bar`.** Every pressure a scenario declares is in bar (six keys across
four node kinds, no `_pa`). The trap that comes with it, named in advance: the controller's
arithmetic is in Pascals, so the setpoint AND the gain both convert at the same
site, and converting one without the other is a factor of 100 000 no type
catches — a gain is a bare `f64` all the way in.

**M8.4's "a level loop must actuate a drain" is too narrow.** What the sign
convention forces is that the actuator is an **outlet of the measured holdup** —
a drain for a level, a vent for a vessel. The demo vents to flare, so the
direction question never arises; throttling the make-up instead is reverse
acting, refused at load in both controllers by design, and is deferred with its
own trigger (DEFERRED E7) rather than smuggled in as a negative gain. The demo
also carries **no PSV** (a shut relief valve is the dead-end shape behind A3) and
its vent must be sized to sit interior at steady state, or the milestone repeats
M8.4's coverage gap where the wired loop never reached its own saturation arm.

**M10.1 landed 2026-09-06** — the measured variable, the vessel arm, the two
config keys and the demo plant. The design note is DESIGN §12, "Corrections from
building it". Seven things to know.

**The seam held, and it held wider than M8 claimed it would.** M8.2 built the
control machinery with one variable in it and asserted it was variable-agnostic.
Not one line of `Engine::run_control_loops` changed — and neither did the
`Controller` trait, either controller implementation, `ControlLoop`, `ControlMode`,
`ControlSnapshot` or `Snapshot`. The whole milestone is two enum arms, four match
arms, two scenario keys and a demo. This project's record is that about half its
predictions are wrong; this one was right, and saying so is part of the record.

**What did NOT hold is one level down: `ControlledValue::error` became unsound and
fork 6 did not name it.** Its doc read "both arguments are the same type by
construction, so a level measurement cannot be differenced against a pressure
setpoint" — **a property of there being ONE variant, not of the type.** With two,
`error(Pressure{5e5}, Level{4.0})` returns a plausible `499996.0`, metres
subtracted from Pascals. It now returns `NaN`, which the engine's existing
finite-output check turns into a diagnosed error. The wider lesson: **a safety
argument resting on a type having one inhabitant expires silently, because the
code does not change.** It also showed the `SetSetpoint` variable guard is
load-bearing rather than cosmetic — it and the tick pass's `setpoint.variable()`
are what keep `error` sound.

**The setpoint's `×1e5` and the gain's `÷1e5` are resolved at ONE site, above the
algorithm match, and that placement is the fix rather than tidiness.** Both `"p"`
and `"pi"` need a gain, so following the existing per-arm shape would have put the
conversion at two sites. Measured: converting one without the other fires no
solver, mass or energy test — the loop stays stable, merely mistuned by five
orders — but it does move the settled operating point, so the demo gates catch it
too. The note's "gate 3 and nothing else" was wrong in the "nothing else" clause.

**Fork 5's reason for expecting a cheap plant is FALSE, and it corrects ledger row
A3.** The fork predicted the demo would avoid `relief_blowdown`'s 920
game-fidelity sweeps because a controlled vent conducts, so its node is not a dead
end. The vent conducts 0.4987 kg/s and the first draft still took **741 sweeps**.
With only the vent line's geometry changed: 2 m × 0.10 m → 741, 5 m × 0.06 m → 34,
the shipped 10 m × 0.05 m → 13. **Dead-endedness is not the mechanism; the
branch's conductance against the vessel's capacitance is.** The shipped geometry
was chosen on gas velocity (20.8 m/s against the placeholder's oversized 5.2), not
to fix this — the sweep count is the consequence, recorded.

**A byte-identity baseline has NO power over the file the slice adds, and that is
how the sharpest mutation escaped.** Giving the new `ControlledValue` variant the
same serde tag as the old one moves **zero** corpus rows on both fidelities and
passes the entire test suite, because the only plant whose bytes change is the one
that is new in the same slice and has no baseline row. The demo then reports
`{"variable":"level","pa":2000000.0}`. Tagging the *existing* variant is caught,
and that is the edit §12's prose describes while its mutation table lists the
other. A wire-form gate now closes it, asserted on the serialized bytes because a
Rust match on `ControlledValue::Pressure { .. }` passes under any tag.

**An identity carried across from another node kind is a hypothesis about that
kind's state vector.** M8.5 measured a tank's pressure and mass "one Euler step
apart"; gate 2 carried that across and FAILED. On a vessel the exact identity is
on the MASS, because `C = V·M̄/(R·T)` is itself a function of a state that moved —
the receiver heats as it fills and the pressures miss by 653.6 Pa, 4.078e-4
relative, **which is exactly `ΔT/T`**. Dividing the temperature out restores it to
1.2e-7. A tank's capacitance analogue is geometry; a vessel's is a state.

**The false sentence survived in a third file, and the write-up was citing its
absence from the diff as a virtue.** `traits.rs`'s `Controller` doc still said the
two arguments "are the same type by construction, so the difference
`ControlledValue::error` takes is always dimensionally honest" — the expired claim
again, on the page the next implementer reads. Corrected after the fact. The
gdext binding was also built and linted behind its feature (`--features godot
--target-dir target/godot`), because a new enum variant is exactly what breaks a
feature-gated exhaustive match and the workspace lint sees none of that crate;
both are clean.

**One of the five specified mutations is not expressible**, and the demo's
counterfactual came out differently from M8.4's. "`measure` reads the solved
pressure" cannot be written — `last_solution` is private to `Engine` and `measure`
takes `&self` on the graph — so gate 1's `NaN` half defends a fault the module
boundary already prevents (fourth time in this project a specified gate had no
power over its own subject). And a parked pressure loop does not run away the way
a parked level loop did: a vent's flow rises with the vessel's own pressure, a far
stiffer feedback than `ρgh`, so it **settles at the wrong number** — 25.197 bar
against the loop's 20.000. The demo's gain bound is also two-sided, unlike M8.4's:
the vent settles interior at 0.558224, so `0.5582` reaches 0 on a one-bar step up
and `0.4418` reaches 1 on a step down. **From here, "runs byte-identical" is
unchanged — all fourteen pre-M10 plants are identical on both fidelities.**

**M9 is CLOSED (2026-09-06), and its scope was solver robustness.** It opened the
way M8 did — with a defect the previous milestone reached and deliberately did not
fix — and slices were scoped one at a time, because what the next one should be
depended on what the last one measured. Six boxes landed: three about the
hydraulic solvers (M9.0, M9.1, M9.2), one that turned the milestone's own
hand-measurement into the `corpus` command (M9.3), and two that spent it (M9.3a,
M9.3b). **Nothing in `docs/DEFERRED.md` was past its trigger** when M9 closed. **That is no longer true: B3 (two phases in the state vector) went past on 2026-09-06** — see its row and the ledger's own summary.

Three things carry forward past the milestone. **The scope moved three times** —
which step to take (M9.0/M9.1), when the solver may stop (M9.2), what an iteration
costs and how many there are (M9.3) — and only the first was foreseeable.
**Every box found its predecessor's write-up wrong** (M8's mechanism false in both
clauses, M9.1's first draft fitted to one divergence, M9.3a's table wrong in every
cell, M9.3b's deferral naming the wrong half), so a write-up composed from memory
or from a plausible mechanism is a hypothesis with the formatting of a result —
run the mutation the sentence implies. And **the next milestone is chosen from a
table**: `refinery corpus` plus `docs/DEFERRED.md`, every open hurdle with the
argument that deferred it, its un-defer trigger, and the measured distance.

**M9.3a landed 2026-09-04** — the bubble-point root finder. The design note is
DESIGN §5, "How fast is fast enough" and "What M9.3a changed". Five things to
know.

**The trigger A1 was sitting past is now written, and it is a FRAME budget, not
a corpus total.** The Godot binding leaves ticking to the scene, which calls
`tick()` from `_physics_process` — 16.7 ms at 60 Hz — and a plant may carry
several columns, so one cascade column gets ~2.5 ms per tick. Per *column*,
because the corpus total is an artefact of which files ship. And read as a ratio
**inside one session**: this machine drifted 1.7× slower in a day, so the probe's
own 12.4 s is not comparable to a number measured later. Now 1.10 ms per tick.

**The probe's own proposed remedy would have broken the solve, and the reason
generalises.** It said "a bisection that stops at the tolerance the caller can
see"; DEFERRED A2 said 22 steps. But the cascade's outer convergence test
*differences two bubble-point outputs*, so the root finder's resolution is a
**noise floor on the test that grades it** — it must stay far below that
tolerance, not meet it. Resolution stayed at the float spacing; speed came from
the method (regula falsi, Illinois weighting, Brent's two-step safeguard, on
`ln Σ K·x`). 60 fixed steps → **15 evaluations**, against bisection's 55.

**The logarithm is the whole speed-up and the safeguard is not — the reverse of
what the first write-up claimed.** Swept independently: with the transform every
safeguard variant costs 15–16, without it none costs less than 38. That first
table was written from memory and **every cell was wrong**, caught by running the
double-revert mutation (predicted 28, measured 38). The safeguard buys the
worst-case bound behind `BUBBLE_POINT_MAX_EVALUATIONS`; the gate deliberately
does not defend it, because a bound tight enough to fire on 16 would be fitted to
one composition. Also: the secant's bracket guard is a **negated conjunction** so
a NaN falls to bisection — the "obvious" rewrite reads identically and poisons
the bracket.

**The expected ~7× was 2.60×, fully attributed rather than shrugged at.** Region
timers in both versions: bubble points 14 288 → 4 686 ms, i.e. **3.05×** not the
4× that 60 → 15 predicts, because each evaluation now costs ~31% more (an `ln`
plus secant arithmetic against a bare midpoint); the rest is the unchanged 14%
floor of the K-profile and Thomas sweeps. `flash.rs`'s bisection is **cleared by
measurement** — once per solve, not once per outer iteration.

**The number that did NOT move scopes M9.3b: 200 000 outer iterations over 6 000
ticks, 33.3 per tick, identical before and after.** This slice made each
iteration cheaper; the warm start makes them fewer, and multiplies all three
regions rather than one. Bubble points are still 77% of the loop.

Two measurement habits from this slice. Wall time was taken **A/B/A/B in one
session with an unrelated plant as a control**, because the machine's drift
exceeds the effect on any single pair. And the movement bound was taken over
**all 600 snapshots, not the final one** — the worst deviation is a transient at
tick 270 (1.31e-14) and the settled value is 3.94e-15, so the endpoint alone
understates it 3.3×. Exactly one of fourteen plants moves, on both fidelities.
**From here, "runs byte-identical" means post-M9.3a identical for
`crude_column_cascade` under both `newton` and `simple`.**

**M9.0 and M9.1 are about the same step**, and reading M9.1 without M9.0 will not
work — M9.1's whole argument is the closed form M9.0 derived, applied to a solver
that had no line search at all.

**M9.0 landed 2026-08-26** — the shut-in stall. The design note is DESIGN §11.
Five things to know before touching the hydraulic solver.

**The mechanism M8 recorded for this defect was false in both of its clauses, and
that is the thing to internalise.** M8 said an unbounded `dQ/dΔP` made the step
enormous and the line search cut it back. Measured: the line search accepted the
full step on all fifty iterations and never halved anything; the shut valve's
conductance is exactly ZERO rather than unbounded; and the branch drop changed
sign every iteration while the residual fell anyway. **A monotone residual history
says nothing about the iterate's path**, because the merit is even in the error
and the error is not — M8 inferred "crawling, not oscillating" from exactly that
and was wrong.

**The comment above the line search was right the whole time; the constant under
it was not.** It said Armijo "rejects [the near-symmetric overshoot] and forces
t ≤ ½". `ARMIJO_C = 1e-4` did not. **A mis-sized constant under a correct comment
is worse than a wrong comment**, because the comment is what stops the next reader
from checking.

**The fix is `ARMIJO_C: 1e-4 → 5e-2` and the arithmetic behind it is a relation
between three constants.** A shut valve orphans a node with exactly one live edge
(F6), so the failing solve is scalar; a full Newton step on `x/√(|x|+ε)` lands on
the MIRROR of the drop, `2ε` nearer the root, while a HALF step lands within `ε`
of the root from any drop at all. A solve therefore stalls iff
`2·eps_dp·max_iter < |Δp₀| ≲ eps_dp/ARMIJO_C`, which is empty iff
`ARMIJO_C ≥ 1/(2·max_iter)`. **`eps_dp` cancels — shrinking the regularisation is
the reflex and is inert.** `armijo_c_closes_the_shut_in_stall_window` asserts the
relation and fires on either half of it, so **lowering `max_iter` reopens the
window** and is a code-review failure without raising `ARMIJO_C` with it.

**Stricter Armijo made the solver FASTER, which is the reverse of the standing
objection.** Worst-case iterations per pass across 6 000 ticks of all fourteen
shipped scenarios fell 11 → 10 against a cap of 50, because rejecting the full
step forces the near-exact half step. Ten of fourteen scenarios stay
byte-identical; four move by at most `7.6e-11` relative on any physical quantity.
**From here, "runs byte-identical" means post-M9.0 identical for those four**
(`fcc_plant`, `knockout_drum`, `leaking_line`, `tank_level_control`).

**Two gates changed shape, and one of them kept a catch nobody was defending.**
`a_branch_shut_in_one_tick_stalls_...` became
`a_branch_shut_in_one_tick_converges_whoever_shuts_it` and asserts the shut
branch's ENDPOINT — zero flow, and the dead leg sitting at the tank's bottom
pressure — because `Ok(())` is passed by any line search that accepts anything.
Righting it kept M8.4's accidental catch of "the control pass moved below the
solve" (now 4.16 kg/s through a branch it says is shut), which closes the tick-order
gap M8.4 recorded as open. **Re-run an upside-down test's catches after turning it
right way up.** The demo's gain gate was re-premised rather than inverted: `0.4`
still clamps, and now recovers.

**M9.1 landed 2026-08-26** — the same step on the OTHER solver. The design note
is DESIGN §11's M9.1 half. Nine things to know.

**The shut valve is not the subject; it is where the defect stops finishing.**
`SimpleFlowSolver` has no step-rejection criterion of any kind — it applies its
full node-wise step unconditionally — so it takes the worst-available step on
EVERY valve node of EVERY plant. On the M8.2 fixture with the drain 20% open and
nothing shut anywhere, it takes 189 sweeps against Newton's 8, and the node still
moving at the end is the FEED valve on the other side of the plant, which takes
exactly 189 whatever the drain does. **"The shut-in stall on the other fidelity"
was the wrong frame, and the probe's own first row said so.**

**Half the mechanism is M9.0's closed form and half is new.** On a shut valve the
drop alternates sign every sweep and falls by exactly `2.0000 Pa`, from
`106 790.90 Pa` — predicted 53 395 sweeps, and raising the cap converges at
**53 411**. On a CONDUCTING valve the same overshoot contracts geometrically
instead, at a rate roughly proportional to the valve's conductance. One mechanism
whose contraction factor reaches 1 as the valve shuts, so **"continuous, not a
cliff" is that limit, not the drop growing** — the drop moves 2.5% across the
whole column and is not what separates the cases.

**The stall window here is unbounded above, and that is a difference in kind.**
With nothing rejecting anything it is `(2·eps_dp·max_iter, ∞)`, so **no `max_iter`
closes it**. M9.0's fork 2 was rejected on cost; here it is not available. And it
is reachable structurally: the cold seed is the mean of the pinned pressures, so
free nodes start ~200 kPa from their roots before anyone touches a valve.

**Damping and the line search are the SAME remedy — measured, both ways.**
Lowering `omega` is the reflex and makes the corpus worst case 3.5× worse
(`relief_blowdown` 868 → 3 035 at `ω = 0.5`, because that plant converges on its
vessel's `−C/dt` term, not on branch conductance). And with the line search in at
`ω = 0.5`, the corpus reproduces the no-line-search `ω = 0.5` numbers EXACTLY,
because the half step already passes its own test. `omega` stays `pub` with a
default of `1.0` and a doc comment that no longer advertises damping as the cure
for stiffness.

**§11's own deferred fork was built here and is inert on the case it was written
for.** The sign-reversal trust region is a big corpus win and reproduces the
shut-in divergence bit for bit, because the mirror step DOES shrink the imbalance
by `2ε` — so "reverses *and* does not shrink" never fires. Not an un-defer trigger
for Newton, but evidence against the rule.

**the closed form does not reach as far as the constants it justifies.** §11's form
says a half step lands within `eps_dp` of the root from any drop, so
`MAX_HALVINGS: 8 → 1` was predicted inert. It DIVERGES M8.0's anchoring plant at
20 000 sweeps. **The first draft of that finding fitted a mechanism to that one
divergence — which is what M8 did — so it was bisected instead:** `2` passes both
the test and the whole workspace, `3` changes nothing. One node needs `t = ¼`, and
**why it is outside the form is NOT measured** (the dead-leg reading — F6 leaves
one live edge so the mirror is exact — is a candidate recorded as a candidate).
So `8` is six halvings of margin and is *not* load-bearing; it is inherited from
`newton_flow`, unjustified there too. Cutting it to `2` would fit a constant to
today's fourteen plants. Also from the pass: a trial evaluation that is not the
step's own function **freezes** the residual (identical to the last digit for
5 000 sweeps) rather than slowing it, caught by two M5-era cross-fidelity tests
and by neither new gate. The reject-all branch is **uncaught**, and the bisection
bounds it — no node needs a step below `¼`, so halvings 3–8 are never the accepted
one, and its 3 281 sites all sit at `|imbalance| ≤ 3.4e-13 kg/s`.

**Lowering `omega` is a CORRECTNESS result, not the cost result the fork argues.**
At `ω = 0.5` the shut-in fixture returns `Ok` with `3.77e-6` kg/s through a shut
branch — inside the solver's own `tol_abs + tol_rel·throughput` and outside the
gate's `1e-6`. A wrong endpoint reported as converged, on the plant the slice
exists for. The 3.5× sweep table is the weaker half of the case for `ω = 1.0`.

**"No shipped scenario runs this solver" is measured, and the first probe lied.**
0 of 14 fire a `panic!` at `SimpleFlowSolver::solve`, with `leaking_line` forced
to `simple` as the control that does. The first run said 14 of 14: `panic!` at the
top of a function makes the rest unreachable, rustc **echoes that source line** in
the diagnostic, and `cargo run` replays cached warnings every invocation. Build
once, then run the binary, and require `panicked at` beside the marker.

**`ARMIJO_C = 5e-2` here is Newton's number and deliberately NOT Newton's
constant, and one M9.0 gate cannot be mirrored.** The two tests are the same test
(the factor of two is the square of the merit, not tuning), but Newton's margin is
five times a bound tied to *its* cap of 50; at 5 000 that bound is `1e-4`, and
`1e-4` leaves a valve 1% open crawling for 1 239 sweeps. Re-swept here: the knee
is between `1e-3` and `1e-2`, and the cost lands entirely on `relief_blowdown`
(6% at `5e-2`, 53% at `2e-1`). The same arithmetic kills a mirrored
`armijo_c_closes_the_shut_in_stall_window` — it would clear by 500× and pass at
almost any constant. **So the shut-in gate alone does not pin this fix**; the
throttled-valve sweep budget beside it is what fails at the slack constant, and
`a_branch_shut_in_one_tick_converges_whoever_shuts_it` now runs on BOTH
fidelities.

**M9.2 landed 2026-08-31** — not the step this time but the STOPPING RULE. The
design note is DESIGN §11's M9.2 section. Six things to know.

**DESIGN §3 has specified "relative mass-imbalance per node" since M1 and the code
never did it.** Both fidelities compared the worst node imbalance against
`tol_abs + tol_rel × the largest flow anywhere in the plant`, so a spur carrying
10 g/s was graded against a 10 kg/s trunk. The fix is one shared function,
`network::grade_nodes`, whose scale is `max |ṁ|` over that node's OWN incident
active edges. Both solvers now stop on one rule structurally, not by convention.
**This framing — code catching up with a written spec — is what licensed changing
a settled decision** instead of re-litigating a constant.

**This is the mechanism behind M9.1's `ω = 0.5` finding.** M9.1 recorded `Ok`
returned with `3.77e-6` kg/s through a shut branch, inside the solver's own
tolerance. The damping was the path; the stopping rule is why the endpoint was
accepted.

**The shipped corpus is not the reachability argument — the fixture is.** Over 500
ticks of all fourteen plants only two exceed the per-node bar at all (2.30× and
1.015×, one node each). On the shut-in fixture at shipped settings the game
fidelity's worst is **127×**. Reach for the fixture when the corpus says "barely".

**The shut valve is the EASY case, for the second slice running.** The dead leg
behind the shut valve cannot discriminate — `1.6e-9` kg/s before AND after. What
discriminates is the valve node while the valve still CONDUCTS, and the observable
is an identity, not a bound: a valve holds no volume, so its two edges must carry
the same flow and their difference IS that node's residual. Worst ratio of miss to
the solver's own promise: newton **1.81 → 0.97**, simple **2.58 → 0.39**, both
failing on the old rule. A ratio of 0.97 is a pass, not a near miss — the bar is
the promise rather than a chosen constant, and near-1 says the criterion BINDS
there. Two dead-end assertions in `tests/invariants.rs` were tightened the same
way, and **predicting both from measuring one was wrong**: one is inert (accepted
flow identical before and after), the other fires under the mutation at `5.97e-8`
kg/s against a `1.00e-8` promise. Two assertions of the same shape on two plants
are two measurements.

**The mutation pass has one uncaught edit and it is fork 1's own.** Reverting to
the plant-wide scale is caught twice; grading only the last node is caught by 12
tests across four binaries. But swapping `Σ` for `max` as the local scale — the
alternative fork 1 rejects — passes EVERYTHING, because `Σ = 2·max` at a two-edge
node so the bar doubles and the valve gate's 0.97 becomes 0.48, and the solve
never spends the extra slack. **Fork 1 rests entirely on the inequality
`max_incident ≤ throughput` — the new rule is never looser than the old — and no
test defends it.** Left open deliberately.

**The gate this slice set out to write was a vessel, and measuring killed it.** A
vessel's accumulation term is deliberately OUTSIDE the scale (fork 2), so a vessel
gate would assert against the one quantity the criterion does not grade. Third
time in this project a specified gate had no power over its own subject.

**Cost is one iteration on one plant, and the `Err` fear was measured away.**
Twelve of fourteen scenarios stay byte-identical over 6 000 ticks; the two that
move do so by ≤ `8e-8` relative on any physical quantity. Worst iterations move
only on `tank_level_control`, 3 → 4. Because proptest generates spurs and dead
legs — the population whose bar collapsed to `tol_abs` — the reachability counts
were compared either side and are identical (chains 238/300, gas 202/205 Newton
and 187/205 Simple, psv chains 196/400). **From here, "runs byte-identical" means
post-M9.2 identical for `relief_blowdown` and `tank_level_control`.**

**M9.3b landed 2026-09-04** — the warm start, and M9.3 closes with it. The design
note is DESIGN §5, "The warm start (M9.3b)". Six things to know.

**Fork 5's word "profile" names the wrong half, and that is the finding.** A stage
cascade iterates stage TEMPERATURES and stage LIQUID COMPOSITIONS at once.
`stage_t` is what reads as "the profile" — it is what `with_seed_offset` perturbs
and what the convergence diagnostic named — and seeding it alone is nearly inert:
38.0 → 35.0 outer iterations per solve, 8%, which vanishes into wall-clock noise.
Seeding BOTH gives **1.006**. The outer convergence test is a CONJUNCTION, and the
composition profile was the binding criterion all along. **The obvious reading
would have shipped the 8% and written the warm start up as measured and
disappointing** — it was caught only because the A/B came back inside the noise
band and the iteration count was measured to find out why.

**The type is the measurement.** `CascadeProfile` holds both halves behind ONE
`Option`, so "temperatures without compositions" — the configuration just
falsified — is unrepresentable. Same shape and argument as the two duties.

**It needed no engine state, and that settled the seam fork.** The previous tick's
`NodeStates` is already an argument to the sweep and its separations are already
keyed by node, so the profile rides `Separation` out and `ColumnPass` back in.
`SeparationModel::separate` keeps `&self` and its "a function of `pass` alone"
contract stays LITERALLY true, because the history is an argument rather than
state on the model. `&mut self` and interior mutability both falsify that sentence
and both put per-column state on the single `Box<dyn SeparationModel>` the engine
holds for every column, which would cross-seed two columns on one plant.

**1.006 iterations per solve needed an adversarial gate, not a celebration** —
converging on the first pass is what a right seed looks like AND what a criterion
that stopped binding looks like. Over 6 000 solves: 38 once (tick 1, before a
profile exists), 1 for 5 998 ticks, one zero-flow return. So it binds when the
seed is ABSENT; what no shipped plant reaches is a seed present and WRONG, and
that is the gate — a light feed's converged profile handed to a heavy feed, which
must return the heavy feed's own cold answer. **Two controls are asserted first**
(the profiles must differ by > 1 K somewhere, the distillates by > 1000× the
tolerance), because without them the gate is passed by a solver that ignores its
seed and equally by one that ignores its feed.

**The fixed-point worry was real, measured, and did not happen.** A warm start
moves the answer by the outer tolerance rather than the ULP — ~8 orders more than
M9.3a — and the cascade feeds tanks that integrate. Over 600 snapshots on both
fidelities: worst move on any quantity above 1e-3 is **7.6e-07**, temperatures
1.7e-08, duties 1.5e-08, pressures at the ULP, tank masses **bit-identical** (a
draw's rate is a mass ratio of the feed, so an inventory never depended on the
profile). The decisive number is that the drift is **flat across all ten deciles**,
first equal to last — a tolerance-ball reseat, not accumulation. The full warm
start is also CLOSER to cold than the partial one, because a better seed converges
nearer the true fixed point.

**One mutation is uncaught, deliberately.** Reverting the liquid half of the seed
fails NOTHING: a half-warm start is still correct, only slow, and no test measures
a cascade's iteration count. (Breaking the convergence test is caught by seven
tests; dropping the seed's shape check by exactly its own gate.) A gate for it
would have to assert a cost rather than a correctness, which is how a fitted test
gets written. The defence is the type: undoing the fix means deleting a struct
field, not forgetting a line. **Result: 4 828.9 → 235.7 ms newton (20.5×) and
4 760.9 → 223.8 ms simple (21.3×) per 6 000 ticks, paired in one session; 37.8×
fewer outer iterations, which is the machine-independent number. From here, "runs
byte-identical" means post-M9.3b identical for `crude_column_cascade` under both
fidelities.**

**M8 is CLOSED (2026-08-26), and its scope was regulation** — control loops, so a
plant holds itself somewhere instead of being held by whoever is sending
commands. Its six slices are summarized below; M9 is unscoped. It opened
with a defect rather than a feature: **M8.0 landed 2026-08-26**, the anchoring
active-set loop (DESIGN §3c), which un-defers M5's FINDING 2 — `network::prepare`
used to freeze the anchored set at the seed compile, so a relief valve whose
`conducts` depends on the pressure *iterate* could be classified stale in either
direction. **M8.1 landed 2026-08-26**: the regulation design note, DESIGN §10,
seven forks argued before any code. The three worth knowing before touching this
milestone — a control loop lives *beside* the graph (`PlantGraph::controls`), not
as a node and not as a field on the actuator it writes; the algorithm is a trait
but the project's **first per-instance seam**, so "rule 2 says trait" had to be
argued rather than inherited, and its impls own STATE where every earlier seam's
are pure; and the loop runs at the TOP of the tick on the *previous* tick's
state, because reading this tick's solve and writing an actuator is an algebraic
loop.

**M8.2 landed 2026-08-26** — the seam itself: `PlantGraph::controls`, the
`Controller` trait (the project's first `Vec<Box<dyn _>>` and its first seam whose
impls own state), `ProportionalController`, the `[[controls]]` table, and the two
loop commands. Five details of the note were corrected while building; three
matter before touching M8.3.

**`initial_output` is deliberately NOT in M8.2, and the reflex that wants it is a
trap.** A proportional controller with no bias shuts its valve completely at
setpoint, which makes `u = u_b + K·e` look obviously right. It is not: fork 5
defines `initial_output` as the loop's *memory* and a P loop has none, and M8.3's
gate reads the P loop's steady-state offset as a signal — a bias makes that offset
a function of how well the bias was chosen instead. So `ProportionalController` is
`u = clamp(K·e, 0, 1)`, the offset is large and honest, and the key belongs to the
PI loop alone.

Two more: the tuning key is **`gain_per_m`**, not the note's bare `gain`, by fork
4's own argument about `setpoint_m` (a gain is `1/m` on a level loop and `1/Pa` on
a pressure loop). And **two refusals the note names have no reachable path today**
— both directions of "the setpoint's variable disagrees with the loop's", which one
`ControlledValue` variant makes unrepresentable — so they are recorded in comments
naming their own expiry rather than shipped as guards nothing reaches.

**M8.3 landed 2026-08-26** — `PiController`, the anti-windup clamp, MANUAL→AUTO
transfer and `initial_output`. Four things to know before M8.4.

**The loop's memory is stored in OUTPUT units, and that is what makes fork 4's
"same arithmetic" claim keepable.** `u = clamp(K·e + b, 0, 1)`, where `b` is the
share of the valve position the integral term owns — not `∫e dt`. Inverting for
"what memory makes the next output be `u`" is then `b = u − K·e`, one private
`back_calculate`, and all three writers of a loop's memory go through it: the
anti-windup clamp, the MANUAL→AUTO seed, and the load-time seed from
`initial_output`. With the textbook state those would have been three formulas.
The integral is also accumulated AFTER the output is computed (explicit Euler),
which is what makes the first update after a seed return the seeded position.

**MANUAL→AUTO reads the measurement FRESH, not `last_measurement`.** Commands are
applied between ticks, so the state at transfer time IS what the next control pass
will measure, and seeding against it makes the transfer exact (measured deviation
zero, against a derived few-ULP bound). The stale read is the reflex, and it was
run as a mutation: it steps the valve by 4.66e-5 — small enough that any bound
picked to "look tight" would have passed it, which is the argument for deriving
the tolerance rather than choosing one.

**Gate 3 as DESIGN §10 specifies it does not discriminate.** "The P loop with an
offset, the PI loop without one" fails as a test because a proportional loop's
offset is `e = u/K`: a big enough gain passes it with no integral term anywhere.
What a P loop *cannot* do is move its output while holding its level. So the gate
asserts that identity — the P half's level move equals its own valve travel over
the gain (0.477229 m measured against 0.477230 m forced), while the PI half moved
its valve further and its level by 0.0013 m.

`initial_output` is required on `algorithm = "pi"`, refused on `"p"` with its own
reason (it names a memory that controller does not have, not an unknown key), and
`"pid"` is now what the unknown-algorithm refusal is tested with.

**M8.4 landed 2026-08-26** — the wired demo, the mutation pass, and a measured
coverage record. Four things to know.

**`scenarios/tank_level_control.toml` is the first shipped file with a
`[[controls]]` table**, and it is meant to be diffed against
`tank_pump_valve.toml`. **A level loop must actuate a DRAIN, and that is forced,
not chosen**: the error is `measurement − setpoint` and the output is
`clamp(K·e + b, 0, 1)`, so a rising level OPENS the actuator — on a fill valve
that is runaway. Every level loop written after this one inherits that. The gain
bound is the plant's own and both sides are run: at the settled operating point
the drain sits at ~0.376, so a +1 m setpoint step subtracts `gain_per_m` in one
tick — 0.25 and 0.35 absorb it, 0.4 hits exactly 0. (M8.4 measured that 0.4
also stalled the solver; **M9.0 fixed that**, so 0.4 now clamps, recovers and
parks on the stepped setpoint. The bound is a tuning bound now, not a stability
one.) The file ships 0.25. M8.2's 0.05/0.1 pair belongs to M8.2's plant and does not
transfer.

**A PI loop cannot slam its actuator at STARTUP at any gain**, which is the
reverse of the worry the roadmap box was written with: the memory is seeded by
`b = u − K·e`, so the first output is the declared `initial_output` whatever `K`
is (gains 0.25 to 20 all survive). The startup step exists only if a file
declares the valve's `opening` and `initial_output` apart. The clamp bound is
reachable only through a setpoint move, so it takes a command to measure and
cannot be read off a CLI run.

**All seven named mutations have now been run, and four of the seven predictions
were wrong.** The two "predicted uncaught" edits were both caught — but read the
mechanisms in DESIGN §10 before trusting either word. "The loop runs after the
solve" is caught only by the test written to FAIL when the solver is fixed, so
**no gate asserts the tick order and that gap is recorded, not filled** — until
M9.0, which righted that test rather than deleting it and thereby kept the catch
as a state assertion.
"`initial_output` ignored" is caught by M8.4's own new startup gate and by the
refusal sweep (the key's range check lives inside `seed_from_output`, so an edit
that stops calling it stops validating too). And "back-calculation dropped on
MANUAL→AUTO" fires the transfer gate alone while gate 4 stays green, falsifying
the table and confirming M8.3's revision of it.

**What the demo does NOT cover was measured, not assumed** — a `panic!` compiled
into each site, the shipped file run for 6 000 ticks, with the load-time seed as
the control that must fire. Not reached: the anti-windup arm (the output never
leaves `[0.194, 0.384]`), `ProportionalController` (no shipped file selects it),
the `Manual` arm of the tick pass, the engine's range backstop. And because the
CLI issues no commands, **the MANUAL→AUTO transfer has no wired exercise at
all** — it is covered by fixtures only.

A control loop can now slam a valve shut between two ticks, and M8.4 found that
**a branch driven to zero flow in ONE tick stalled the Newton solver**. That was
never the loop's defect — `Command::SetValveOpening` writing the identical
endpoint failed identically — and **M9.0 fixed it in the solver** (see the M9 box
below). A level loop no longer needs a gain gentle enough to avoid clamping; it
still wants one, for tuning reasons.

(The census of scenario files that declare a `[[controls]]` table lived here,
then in `CLAUDE.md`; it is no longer kept by hand. `grep -l '\[\[controls\]\]'
scenarios/*.toml` lists them.)

**M8.5 landed 2026-08-26, and M8 is closed** — `Snapshot::slate`, so a frontend
can turn a tank's mass into a fill level. Four things to know.

**The regression anchor moved by exactly one key, and it moved on purpose.**
`slate` is the first field on `Snapshot` with neither `serde(default)` nor
`skip_serializing_if`, so every scenario's JSON changed. That is not an oversight
copied from the wrong pattern — it is the discriminating argument: `controls: []`
and `column_duty: None` are *true statements* about a plant, while an empty slate
is impossible (`Slate::new` refuses one), so a `default` would let an old document
deserialize into a snapshot claiming the plant has no components. Measured rather
than predicted: strip `"slate":[…],` from each of the 28 after-runs and all 28
reproduce their before-file byte for byte. **From here, "runs byte-identical"
means post-M8.5 identical.**

**A tank's pressure cannot gate its density, and that is algebra, not a gap.**
The obvious independent check on a published density is the tank's hydrostatic
head — but `P − P_ATM = ρ·g·h = ρ·g·(m/ρA) = m·g/A`, so the density cancels
exactly and a snapshot shipping `cp` in the density slot would move both sides
identically. Every other candidate cancels the same way: a density is observable
only through a *volume*, and the only volume a scenario declares is
`initial_level_m`. So the load-time level is the **single** anchor outside the
code, it exists only at tick 0, and that is what the real gate reconstructs. The
impossibility is kept as an assertion rather than dropped, because it is the
first thing the next person will reach for.

**A snapshot's tank pressure and its tank mass are one Euler step apart** — 0.67
Pa, 8.6e-6 relative, found by that assertion failing. The tick is solve →
transport → unit dynamics, so the pressure came from the mass at the *start* of
the tick. Nothing is wrong; a frontend drawing a level reads mass, the fresh one.

**A tank's component densities are never `null`, so the scene needs no fallback.**
The `Option` exists because a slate may carry gas cuts; it is unreachable down
the fill path because the loader refuses a gas-phase tank and a gas holdup is a
`vessel`, whose state is a pressure. Swept across all fourteen shipped files.
The wired demo (`leaking_line.toml`) is water-only, so **it cannot exercise
mixing at all** — that is covered on `crude_column.toml`, whose naphtha tank
becomes a real two-component mixture as it fills.

M1–M7 are closed: flow network, heat, crude + simple column, reactor, gas and
pressure realism, damage + the Godot frontend, and the complex column. **M7 closed
2026-08-18** — the design note (DESIGN §5, "Complex column (M7)"), the
`SeparationModel` seam (M7.1), `ThermoModel::k_value` with `TroutonThermo` and the
Rachford–Rice flash (M7.2), `StageCascade` with `CascadeSpec` and stage-located
`ColumnDraw`s (M7.3), and M7.4's three slices: tray temperatures through
`edge_temperature_at`'s column arm (a), `dh_vap` plus both duties plus the
saturated-liquid feed guard (b), and I7's cascade arm plus
`scenarios/crude_column_cascade.toml` (c).
The note's verdicts are decisions, not results — M7.1 corrected its call shape,
M7.2 its signature and gate structure, M7.3 found that the constant-α model
M7.2 shipped *for* the cascade could not have driven one stage of it, M7.4a
falsified its own gate's justification with a mutation, and **M7.4b and M7.4c each
found that a gate their own box specified cannot exist** — both because the
reboiler duty is *defined* to close the balance those gates would have checked it
against. The rule that came out of it: a quantity defined to close a balance can
never be gated by that balance, so check an invariant's two sides are computed by
independent paths before writing it.

The two column demos are a PAIR and are meant to be diffed:
`crude_column.toml` (cut-point) and `crude_column_cascade.toml` (cascade) run the
same crude into the same three tanks at the same three rates, and differ only in
what the separation model does with it. `crude_column.toml`'s first cut sits at
155 °C so the heavy naphtha boils inside its smearing ramp — that is deliberate
and is the only exercise `smearing_k` gets on a wired plant.

A cascade column is specified by ratios only: `reflux_ratio` (molar, internal)
plus one `draw_ratio` per draw (a **mass** fraction of the feed, with the bottoms
left over). An absolute product rate in kg/s is inadmissible — it re-runs the
failure that killed M3.2's prescribed-draw column. `N` counts the reboiler and
excludes the total condenser.

The two separation fidelities carry mutually exclusive config, enforced at load in
both directions: `up_to_c` + `smearing_k` are the splitter's, `stage` +
`draw_ratio` + `phase` + `[cascade]` are the cascade's. Adding a knob to one means
refusing it on the other.

Draw temperatures are real tray temperatures and are **read**:
`energy::column_draw_at` is the single owner of "which draw is this edge" for both
composition and temperature, so the two fields of a draw always come from the same
draw. A cascade column is therefore not enthalpy-neutral, and M7.4b's duties are
what close its external energy books.

**The two duties are not symmetric, and treating them as if they were is the
mistake to avoid.** The condenser duty is its own exact envelope; the reboiler
duty is *defined* as the condenser duty plus the column's external sensible
balance. That asymmetry is forced: constant molar overflow leaves every interior
stage with an energy residual, so a locally-exact reboiler duty would disagree with
the column's own balance by several times the quantity that balance measures. Two
consequences to hold on to — the reboiler duty carries the formulation's error and
the condenser duty does not, and **"the difference equals the external balance" is
a tautology, not a gate** (DESIGN §5, M7.4b correction 1).

Both duties are `Option<Watt>`: `None` from the cut-point splitter, which has no
such equipment, and `Some` from the cascade — including `Some(ZERO)` for an idle
column, which is an answer rather than an absence. `NodeSnapshot::column_duty`
carries that outward and is skipped when `None`.

A cascade's feed must be a **saturated liquid** and this is now refused rather than
assumed, in both directions, outside a window of `ε·Δh_vap/c̄p` at `ε = 1%` — about
±1.2 K on the M7.3 slate, wider on a heavier one. Any new cascade scenario has to
be built with its feed on the mix's bubble point at the column's pressure; that is
a design input for the file, not something to tune afterwards.
