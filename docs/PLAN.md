# Work plan — M64 and every open ledger row (written 2026-10-17)

A to-do list in the order the work is meant to be taken, asked for after M63
closed: "a plan that includes M64 and all open items, as a todo list; how you
order them is your call". Each row is a `docs/DEFERRED.md` id; the ledger keeps
the detail, the trigger and the distance. Nothing here opens a milestone by
itself: each phase's items are still taken one milestone at a time, probed
first, with the user's decisions on their forks (CLAUDE.md, "Current
milestone").

**How it is ordered.** Steam stripping first, because it is the milestone
already planned and its ground is laid. Then the work that builds on the
column it leaves (the wetter crude, the screen, the solve's cost). Then the
frame budget, because a game that misses its frame is broken in a way a
missing feature is not. Then the boiling/water leftovers, while M62–M64 are
fresh. Then cheap hygiene, then fidelity by area, ending with the rows that
need a decision against a standing rule before anyone starts them.

Ticked when it lands; struck when the user rules it out.

## Phase 1 — M64, steam stripping (C7)

- [x] **M64.0 — probe and design note** (DESIGN §69; done 2026-10-17, `tools/steptime/prototypes/m64_0/FINDINGS.md`): on a copy of
      `crude_unit_steam.toml`, the reboiler held at zero and the reflux the
      result, no vapour floor below the feed. Measure: the bottom temperature
      against the steam rate; the heat the column needs, and whether the coil
      fires it under its 550 °C tube limit; whether the transfer line chokes;
      the cold start from a column that keeps its reboiler; whether C14's fixed
      pumparounds bite; the cost per tick and of tick 1.
- [x] **M64.1 — the reboiler held at zero** (done 2026-10-17, DESIGN §69.7): a held reboiler duty, the reflux an
      unknown in Newton; no floor below the feed on such a column only (C8's
      below-the-feed clause, M62 decision 1); every shipped plant byte-identical.
- [x] **M64.2 — the cold start** (done 2026-10-17, DESIGN §69.8): a continuation
      from a column that keeps its reboiler, its side cooling derived from the
      feed (the user's decision); one continuation a tick.
- [x] **M64.3 — the stripped crude unit** (done 2026-10-17, DESIGN §69.9): a new plant file, gated on the probe
      and the first law; both fidelities; and a move that forces the held
      column's fallback (its warm start failing mid-run), owed by M64.2.
- [ ] **M64.4 — the column screen's fourth plant**, its keys measured; the
      godot-feature clippy.

## Phase 2 — the column after M64

- [ ] **C14** — the wet crude unit on a wetter crude: the pumparounds against the
      charge.
- [ ] **C15** — the stripped crude unit started from a cold plant: the furnace
      lit, the column on its reboiler, the steam cut in and the reboiler handed
      down to zero as the feed heats (asked for 2026-10-17; M64 starts hot).
- [ ] **C12** — the column screen's keys as a box set by its worst corner.
- [ ] **C11** — Newton's Jacobian by forward differences (cost; feeds Phase 3).
- [ ] **C9** — a pumparound as a real liquid circuit, not a side cooler.
- [ ] **B3** (open clause) — a flashing feed into a column, a partial condenser,
      a vapour side draw; M7's three refusals.
- [ ] **C1** — tray hydraulics: pressure drop, weeping, flooding — and with it
      **C8**, the 20 % vapour floor above the feed, a game value standing in
      for C1.
- [ ] **C4** — Murphree efficiency per tray.
- [ ] **C3** — non-ideal K (activity coefficients).
- [ ] **C2** — column holdup and tray dynamics.
- [ ] **C5** (open clause) — the `"cascade"` model's saturated-liquid feed
      refusal (the heat-balance model took the row at M60).
- [ ] **C10** (what is left) — the cold warm-up's start aids, convergence aids
      rather than physics.

## Phase 3 — the frame budget

- [ ] **A29** — a boiling coil plant re-solves every tick of its coil's drift.
- [ ] **A24** — the gas-lock story's ticks past the frame on the game solver.
- [ ] **A25** — the dying pump's 19 sweeps a tick; the engine's sweep unmemoised.
- [ ] **A27** — the quench cascade's moving valve on the game solver.
- [ ] **A19** — the game solver while a check valve rides its band.

## Phase 4 — boiling and water leftovers (M53–M63)

- [ ] **B63** with **B47** — a valve and a pipe choking on a part-liquid boiling
      stream (built and measured in M62.6c, then deferred).
- [ ] **B66** — a gas heated by a valve's `Δp/ρ`, a line-flash vapour not.
- [ ] **B61** — free water settling at once in a tank.
- [ ] **B65** — dissolved water, emulsions, desalting, salt.
- [ ] **B51** — two latent heats meeting in a flashing tank.
- [ ] **B58** — a gas valve's `γ` and a line's phase read the stored composition.
- [ ] **B60** — off the line flash, a zero-volume node's state a tick late.
- [ ] **B52** — a zero-volume node on a pump's suction carries a tick's lag.
- [ ] **B57** — two pumps joined by one pipe refused on a flashing plant.
- [ ] **B59** — the gas pocket fills at one rate whatever the vapour.
- [ ] **B54** — the two-phase head multiplier reads the inlet's vapour.
- [ ] **B55** — RELAP5's fully degraded head taken as zero.
- [ ] **B49** — phase slip, and a condensing wall.
- [ ] **B11** — a pressurised two-phase holdup.

## Phase 5 — cheap hygiene: tests that defend, APIs that cannot be misused

- [ ] **F6** — a trip-held valve's refusal reads as debug text.
- [ ] **A11** — `max_iter` and `omega` public.
- [ ] **A13** — `CascadeProfile::liquid` a bare `Vec<Vec<f64>>` in `core`.
- [ ] **A6** — `max_incident ≤ throughput` undefended by any test.
- [ ] **A16** — the order of group shifts within a level undefended.
- [ ] **A17** — a group step's baseline and trials from two compiles.
- [ ] **A10** — the Simple solver's cold seed.
- [ ] **A5** — a terminal consumer's single edge held to `tol_abs`.
- [ ] **A4** — `tol_rel` never re-swept against a node's own traffic.
- [ ] **A7** — `MAX_HALVINGS` margin on M8.0's anchoring plant.
- [ ] **A8** — Newton's 2 Pa steps toward a shut branch.
- [ ] **A9** — the sign-reversal trust region.
- [ ] **A15** — a stagnant node's temperature depends on the fidelity.
- [ ] **A18** — Newton's Jacobian freezes the gas density and fold.
- [ ] **A12** — dense LU in the Newton flow solver.
- [ ] **B24**, **B25**, **B26** — the shaped heat capacity: no shipped caller, no
      constant twin, water without a shape.

## Phase 6 — hydraulics and gas fidelity

- [ ] **B2** — Colebrook/Haaland friction in place of a fixed factor.
- [ ] **B4** with **B14** — real-gas `Z`, gas `cp(T)`, a second equation of state.
- [ ] **B15** (open clause) — the liquid clause.
- [ ] **B22**, **B23**, **B27** — a cascade column, a compressible valve and a
      pipe's outlet under a shaped heat capacity.
- [ ] **B41** — a gas hole with the liquid `Cd`.
- [ ] **B45** — NPSH3 one number per pump.
- [ ] **B5** — pump efficiency heating.
- [ ] **B19** — a condenser colder than ambient.
- [ ] **B10** — latching a cavitation transient.

## Phase 7 — tanks, relief and damage

- [ ] **B13** (open clause) — the flare.
- [ ] **B6** (open clause) — a pop valve in liquid service, or with no vessel behind it.
- [ ] **B20** — a vent chain deeper than two.
- [ ] **B21** — recovered condensate rejoining the process.
- [ ] **B30**, **B31** — overflow to a destination; a finite overflow.
- [ ] **B32** — a roofed tank that pressurises when full.
- [ ] **B34**, **B35**, **B36**, **B38** — the dry tank: a recycle through it, a
      fire on it, the game solver's clamp, the wet pass every tick.
- [ ] **B42**, **B43**, **B44** — burn-out at a lumped temperature, round tube
      limits, one flame for every leak.
- [ ] **B40** (open clause) — a commanded fire on a dry furnace.
- [ ] **B7** then **B33** — air ingress and combustion; then what a spilled pool
      does.

## Phase 8 — regulation and trips

- [ ] **E3** — derivative action.
- [ ] **E5** — actuator dynamics and deadband.
- [ ] **E1b** (open clause) — flow control, and §21's two temperature loops.
- [ ] **E10** — a cooler's outlet loop with no lag.
- [ ] **E4** — split-range, override, feedforward.
- [ ] **E7** — reverse action.
- [ ] **E11**, **E12** — reverse flow under a flow loop; a flow measured away from
      its valve.
- [ ] **E21** — a loop's valve more than one pipe from its holdup.
- [ ] **E17**, **E18**, **E20**, **E29** — cascades: deeper chains, other
      pairings, a held primary's memory, a held loop in a cascade.
- [ ] **E9** — the tick-0 rule elsewhere.
- [ ] **E13** (open clause) — a trip on a furnace's or cooler's outlet.
- [ ] **E14** (open clause) — a cooler as trip equipment.
- [ ] **E15** (open clause) — the rest of a safety system: maintenance bypass,
      voting, trip delay, first-out, startup bypass.
- [ ] **E23**, **E26**, **E27** — the check valve's disc: preload and closing,
      its own drop, a published `x_T`.

## Phase 9 — reactors

- [ ] **D6** — an axial temperature profile in a bed.
- [ ] **D19** — thermal cracking past about 450 °C.
- [ ] **D9** — deactivation, wall heat loss, bed pressure drop.
- [ ] **D2** (open clause) — residence time on the held reactor.
- [ ] **D5** — coke-on-catalyst deactivation.
- [ ] **D3**, **D4** — a reactor separating coke; the regenerator loop.
- [ ] **D10**, **D12** — a trip that stops a bed's feed; a failed wall relighting
      its preheater.
- [ ] **D14** — oil and hydrogen at their own pressures through a bed.
- [ ] **D16**, **D17**, **D21** — the recycle loop: light ends and purge,
      compression and surge, a vented loop fed from the flare.
- [ ] **D18**, **D20** — the heat of hydrogenation; a quench's cold check.
- [ ] **D11**, **D13**, **D15**, **A28** — sourced numbers in place of game values:
      kinetics, the bed wall, the hydrogen order, the hydrogen factor's floor.

## Needs a decision against a standing rule first

- [ ] **B8** — acoustic and pressure-wave dynamics. CLAUDE.md's "quasi-steady
      hydraulics, no pressure waves" is a decision made; this row reopens it.
