# M74.4 — the holdup plant's keys: the cost sample (DESIGN §79.7)

`crude_unit_holdup.toml` on the column screen's keys (the tray plant's four
and a reboiler: held at a duty, or free). Before any table, the cost and the
settling time sampled (the advisor's and §79.2's "planned before launching").

## The files

- `m74_4_grid.rs` — the driver (M73.4's, on this plant, with `reb`/`free`,
  the levels and limit ticks after the last move, `CHECKS` readings). Copy it
  into `crates/godot-ext/tests/`, register it in `suite.rs`, build
  `cargo test --release -p refinery-godot-ext --test suite --no-run`, run
  `run.sh` (Linux, `nice`). Not left registered.
- `sample.py` — random single jumps (seed 7); `sample.specs` the 40 run.

## The sample (HEAD 4ad45f4, 4 threads, Linux)

40 random jumps over furnace 0–40 MW, reflux 0.5–15, the loops 0–200 % and
−60…+60 K, the reboiler free or held at 50–130 MW; each jump at tick 1 000,
run 24 000 ticks after it (40 minutes of plant time), read at 2 000, 4 000,
8 000, 12 000, 16 000 and 24 000.

- **Cost**: 26.9–45.9 s a run, median 28.7 s — about 1.2 ms a tick. 20 minutes
  of wall time on 4 threads for the 40.
- **Every one ran.** None failed — on the tray plant (key 7) about 5 % of the
  jumps on its grid fail, most on the jump's own tick; one of these
  (reflux 0.5, the loops at 50 % and +20 K, free) is outside key 7's table.
- **Settling**: the products within 5 % of their 24 000-tick flows by
  8 000 ticks after the jump on 4 of 40 runs, by 12 000 on 11, by 16 000 on
  27; several still moving 15–37 % between 16 000 and 24 000 (the held
  reboiler high against the feed's heat: the base's loop swinging slowly).
- **The physical limits**: 3 runs end with the base dry (the reboiler held
  at 100–130 MW against a reflux of 1–1.5 and the furnace at 30–35 MW); 6
  more touched dry on the way (the base down to 0.07–0.2) and came back; the
  drum dry briefly on 4. None ran full.
