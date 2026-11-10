# M74.4 findings — the holdup plant's keys (2026-11-10)

The record is DESIGN §79.7 (the tray-short fix, dry trays shown) and §79.8 (the
batches, the box, the reboiler's floor at 80 MW). This folder holds the tools:

- `m74_4_grid.rs` — the driver. Copy into `crates/godot-ext/tests/`, add
  `#[path = "m74_4_grid.rs"] mod m74_4_grid;` to its `suite.rs`, build
  `cargo test -p refinery-godot-ext --release --test suite --no-run` into a
  scratch target dir, run the exe `m74_4_grid --nocapture --test-threads 1`
  with `PLANT=holdup|trays`, `SPECS` or `SPECS_FILE`, `JOBS`, `RUN`.
  A spec: `key=value+…@ticks>…`, optionally `~N` (ticks after the last move);
  keys furnace, reflux, flow, ret, reb (held MW), free, rebstep (T/G). The
  driver applies no key range: `rebstep` walks below the screen's floor.
- `specs_sample.py` — random single moves. `specs_checks.py holds|free|walks`
  — the corners, the long free runs, the walks.
- `analyse_sample.py` — a batch's failures by kind, reboiler and reflux; the
  runs ending at a limit or with a dry tray; the slowest tick.

Cost: about 1.4 ms a tick on 14 jobs, so an hour holds ~2 800 runs of 13 000
ticks.
