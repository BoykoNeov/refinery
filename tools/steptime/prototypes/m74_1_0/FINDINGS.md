# M74.1.0 — which stream holds each level: the probe, 2026-10-22

After M74.0 the user chose each tray's liquid **following the flow over it**,
the reflux drum's and base's sizes **declared per plant**, and **a new file
copied from the tray plant** (DESIGN §79). Following the flow, the column
stores and gives up mass as the keys move, so its products no longer each
come to a fixed share of this tick's feed: a stream must hold the reflux
drum's level and one the base's (§79.1). This probe measures the trays'
liquid against their flow, the mass the column stores on each key move, and
one scheme's level loops. No engine code changed.

## The probe

- `probe_holdup.patch`: M74.0's recorder (`../m74_0/`) with each tray's whole
  head, dry-plate head, residual head, loss under the downcomer's apron, weir
  length and passes added. Driver and run as M74.0's (`../m74_0/`), the same
  ten runs (`steps.out`).
- `levels.py <steps.out> [drum min] [base min]`: total moles per stage, no
  compositions (M74.0 has them):
  - **a tray's liquid out over its weir from what it holds**: Sinnott's heads
    as M73 builds them (`trays.rs`, `sieve_tray`), at the vapour after the
    move: `M(L) = ρ·[A_a·(h_w + h_ow) + A_dc·(h_w + h_ow + h_t + h_dc)]`, the
    crest `h_ow ∝ L^⅔` (Francis), `h_t` the tray's whole head, `h_dc ∝ L²`
    under the apron; checked to read the engine's backup at the recorded load
    (1e-9 m), and inverted. The side draws and loop draws are fixed flows out
    of the liquid leaving; what is left goes down.
  - **the vapour has no holdup**: each stage's is the converged one after the
    move plus the reboiler's move from its settled value (constant molar
    overflow on the change).
  - **scheme A** (DESIGN §79.1; Kister, *Distillation Operation*, 1990,
    Table 16.1 p. 497, Fig. 16.4e): the distillate holds the drum's level,
    the reboiler the base's; the bottoms and side draws at the settled flows,
    the reflux R times the distillate.
  - **the level loops**: proportional-integral on the level's departure from
    normal (a share of the normal holdup), the output a share of the settled
    flow; three tunings, GAME VALUES: gain 1 and 600 s, 2 and 300 s, 0.5 and
    1 200 s. Drum 5 minutes of the distillate, base 5 of the bottoms at
    normal level (M74.0's, Kister's).
  - Starts from the state before the move (its holdups, its flows), the new
    setting from t = 0, explicit Euler at 0.1 s for an hour. Seven moves:
    the reflux to 2, 3, 5, the furnace to 25 MW, the loops' flow ×0.5, ×1.5,
    their return −30 K. 26 s in all.
- `levels.txt`: the output.

## Findings

1. **A tray answers its flow in 5–7 seconds** (`dM/dL` at the file's loads,
   2.0–5.0 t each): a step in the reflux reaches the base through the twelve
   trays' lags, about 77 s — fast beside the compositions' 13 minutes (M74.0).
   Explicit at 0.1 s it is fifty steps inside a tray's lag.
2. **What a key move stores or gives up**: 3.5–7 t of the column's 60 t
   (the reflux to 2: −7.0 t; the loops' flow ×1.5: +7.2 t), most of it on
   the trays (more liquid over a weir stands higher on its tray and in its
   downcomer); the furnace's 25 MW −0.2 t.
3. **Scheme A runs, at every tuning tried, and lands on the column's settled
   flows** — the distillate and the reboiler within 0.0003 of their settled
   values after an hour at gain 2 and 300 s; within 1–3.5 % at the slowest
   tuning, still moving.
4. **How far the levels swing and the streams move** (gain 2, 300 s): the drum
   −17 to +22 % of its normal holdup, the base −17 to +26 %; the distillate
   0.63–1.53 times its settled flow, the reboiler 0.40–1.94 times — the
   reboiler swings hardest, because it alone puts the base's mass right and
   the column's stored mass passes through the base. At gain 0.5 and 1 200 s:
   levels to −44 / +60 %, flows settled in over an hour. At gain 1 and 600 s,
   between.
5. **The flows settle in 17–38 minutes** at gain 2 and 300 s, 23–60 at gain 1
   and 600 s — the same scale as the compositions (M74.0: the top product 90 %
   in 8–15 minutes). The level loops and the compositions both shape what the
   player sees; the build measures them together.
6. **No tray ran short of its draws** in any run.
7. **Scheme B, first argued and not run** ("its level loops would behave as
   A's") — **wrong; run below** (findings 9–12). What differs besides: the
   player sets the reflux ratio and the reboiler's duty, and the distillate
   and bottoms fall out, every product moving with every key.
8. **A held reboiler under A**: the reboiler held by command (M66, any
   column) leaves the base's level with no stream holding it. Either the
   bottoms takes the base's level while the reboiler is held (Kister's
   Fig. 16.4a pairing for the base), or holding the reboiler is refused on a
   column with holdup. The tray plant's screen key has no reboiler key.

## Scheme B run, and the screen's biggest jumps (added the same day)

The advisor doubted finding 7: under A the base is put right by the boilup,
hundreds of kg/s; under B by the bottoms, about 29 kg/s, while a key move
stores or gives up several tonnes within a minute or two. `levels.py` now runs
both schemes (`[scheme]`, the level loops' outputs a multiple of the flows
before the move, as §79.2's "the flow the first tick settled on"); B holds
the reboiler's boilup at its value before the move, and its end point is
approximate (the flows after the move are A's closure). The driver re-run
with the screen's biggest one-tick jumps (`steps_big.out`: the reflux to 0.5,
1, 8, 15, the loops' flow to 0 and 2, their return ±60 K; 8 runs, about 20 s).
Outputs `levels_AB_base5.txt` (both schemes, two tunings, Kister's 5-minute
base), `levels_B_base10.txt`, `levels_B_base20.txt` (B, gain 2 and 300 s).

9. **Under B the base swings hard.** Kister's 5-minute base, gain 2 and
   300 s: the moderate moves swing it −69 to +73 % of its normal holdup (the
   loops' flow ×1.5, the reflux to 2), the bottoms pinned at zero for part of
   the way; the big ones empty it. A at the same base and tuning: within
   ±21 % on every moderate move.
10. **Some settings have no steady state under B.** With the reboiler held
    and the reflux cut to 0.5 or 1, or the loops stopped, more boils off the
    base than reaches it: the base empties whatever its size (the run's
    "stored" −145 to −317 t after an hour). Real: the column dries from the
    bottom unless the reboiler is turned down. The engine needs a rule for an
    empty base (and drum), not an error.
11. **A larger base tames B's moderate moves**: 10 minutes of the bottoms,
    within ±42 % (the loops' flow ×2 −72 %, the reflux to 8 empties); 20
    minutes, within ±38 % (the reflux to 8 −63 %). Ludwig's rules: 5–10
    minutes for a product feeding another unit, 30 minutes half-full for a
    drum feeding a furnace (Vol. 1, p. xxiv) — a real crude column's residue
    feeds the vacuum unit's furnace.
12. **A too has a limit**: the reflux to 15 (every tray flooded) stores 46 t,
    the reboiler pinned at zero, and the base fills past full. A faster tuning
    (gain 4, 150 s) halves A's swings and barely moves B's (the bottoms is
    pinned at zero).

## Not checked

- **The vapour as a perturbation**: constant molar overflow on the
  reboiler's move, the vapour's shape jumping to the settled one at t = 0. The
  engine's stage energy balances decide it when built.
- **The vapour's effect on a tray's liquid** (the dry-plate head in the
  downcomer backup) is held at the settled vapour.
- **The reboiler's own limit**: it swings to 1.94 times its settled duty
  here; the engine's reboiler has no capacity today.
