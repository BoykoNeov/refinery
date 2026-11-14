# M75.4 — the overfill plant's stories and its key range (DESIGN §80.7)

`m75_4_probes.rs` holds the probes, not compiled. To run one, append it to
`crates/scenarios/tests/column_overfill.rs` (it uses that file's helpers,
`to_corner` among them), then run
`cargo test -p refinery-scenarios --test suite column_overfill::<probe> -- --ignored --nocapture`
at below-normal priority. Every run starts from the shipped
`scenarios/crude_unit_overfill.toml` (Kv 5 000), 600 ticks at rest.

## What the screen's keys allow (`probe_overfill_stories`, `probe_drain_story`)

`STORY=` sets the story; each runs for 20 minutes.

| story | result |
|---|---|
| `drain` (drain shut, reflux 0.5, reboiler held) | 1.8 bar throughout; the drum 0.41–0.78, never full |
| `probe_drain_story` (reboiler free, drain shut, reflux 0.5) | 1.8 bar; the drum 0.41–0.78, never full |
| `reflux_high` (drain shut, reflux 15) | 3.35 bar at 20 minutes and still rising; no relief |
| `reboiler_drain` (held at 120 MW) | settles near 2.4 bar |
| `total` / `total_drain` | relief at 349.7 s, 3.58 bar held, 18–26 kg/s |

The drum never overfills from the keys, so a shut drain shows nothing on
its own. That is why the user chose a cooling-cut command (§80.7).

## The key box's corners (`probe_box_corners`)

There are 48 runs. Each is a corner of the holding plant's box: the furnace
0 or 40 MW, the reflux 0.5 or 15, the reboiler held at 0 or 215 MW or free,
and the loops either stopped and 60 K warm or doubled and 60 K cold. Each
corner is held for 5 minutes, then the cooling is cut or total reflux put
on for 10 minutes, then undone for 5. `THREADS=12`.

- `box.txt`: the overfill plant before the fix. **38 ran, 10 refused.** Seven
  distinct corners failed, most of them within 2 minutes of the jump.
- `box_plain.txt` (`PLAIN=1`): the holding plant with its steel and no
  condenser limit, with no events. **24 of 24 ran.**
- `box_big.txt` (`BIG=1`): two failing corners on a copy with `margin = 10`,
  so the pressure never moves. **Both ran.** This control points at the
  moving pressure.
- `box4.txt`: after the fix. **48 of 48 ran.** Eight were held at the
  ceiling with its alarm for a while.

## Random walks (`probe_random_walks`)

Each walk is a random key every minute for 30 minutes: a press or a jump on
any of the box's keys, with the cooling, total reflux and the drain toggled
among them. `RUNS`, `MINUTES` and `THREADS` set the size; `SEED=n` replays
one walk.

- `walks.txt`: 24 walks with the midpoint-only retreat. **22 ran; seeds 8
  and 16 were refused.** In each, a mid-column stage had stopped boiling.
- `walks2.txt`: 48 walks with the final retreat, which goes halfway towards
  each end of the bracket in turn. **48 of 48 ran.** Seventeen were held at
  the ceiling for a while.

Seed 16 is replayed as
`column_overfill::slow_a_walk_whose_falsi_and_midpoint_both_stall_runs`.

## Where the trials failed (`probe_one_corner`, with a temporary `PDEBUG` print in `stepped_pass`)

On the corner at furnace 0, reflux 0.5, reboiler held at 0, the bracket's
first trial, 413 Pa above the start, stalled the column's step. The answer
lay about 25 Pa up. On seed 16, the falsi trial (347 271 Pa) and the
bracket's midpoint (347 138 Pa) both stalled. The answer was near the
previous tick's 346 729 Pa.
