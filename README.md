# Refinery Simulator

Headless, deterministic oil-refinery simulation engine in Rust with swappable
fidelity (game ↔ research), consumed by a Godot 4 game, a CLI batch runner,
and future dashboards.

- **Play:** double-click `Refinery.cmd` — it builds the simulator library and
  opens Godot (4.7+, on PATH) on a menu of every screen and plant; Esc in a
  screen comes back to the menu, F11 is full screen
- **Start here:** [CLAUDE.md](CLAUDE.md) — project rules and conventions; the
  latest milestone's close-out is at the top of [docs/MILESTONES.md](docs/MILESTONES.md)
- **Architecture & physics:** [docs/DESIGN.md](docs/DESIGN.md) — every design
  note, written before its slice was built and corrected after
- **Milestones:** [docs/ROADMAP.md](docs/ROADMAP.md) — the current one is
  named at the bottom of [CLAUDE.md](CLAUDE.md), the only place that is kept
  up to date
- **Towards a game:** [docs/GAME.md](docs/GAME.md) — what a player would need
  that is missing, and the visual direction; [docs/plans/](docs/plans/) holds
  a plan for each item
- **Open hurdles:** [docs/DEFERRED.md](docs/DEFERRED.md) — every deferred
  physics and numerics item in one ledger, with what un-defers each and how far
  the corpus sits from that trigger

## Workspace

| crate | what it is |
|---|---|
| `refinery-core` | units, plant graph, engine loop, solver traits, snapshots. No heavy deps, no I/O beyond serde |
| `refinery-solvers` | the trait implementations: two flow solvers, thermo, kinetics, the stage cascade |
| `refinery-scenarios` | the TOML plant format (`schema`), the engine builder (`build`), and the load-time refusals (`validate`) |
| `refinery-cli` | headless runner and the corpus probe |
| `refinery-godot-ext` | the one crate that knows Godot exists; its binding sits behind an off-by-default feature |

## Commands

```sh
cargo test --workspace                                  # must pass before any commit
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all

# one plant, JSON-lines snapshots
cargo run -p refinery-cli -- run scenarios/tank_pump_valve.toml --ticks 1000

# every plant: worst solver iterations, wall time, and a fingerprint per plant
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000 --solver simple
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000 --out before.json
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000 --baseline before.json
```

The last pair is how "runs byte-identical" is claimed: record the rows before a
change, compare after, and the command exits nonzero if any plant moved. With
or without a baseline, a plant that fails to load or fails a tick also exits
nonzero — which is what makes the CI steps that run it without one worth having.

Status: 71 plants, two flow solvers, six Godot screens (five hand-drawn
stories and a map that draws any plant). The current milestone is the last
section of [CLAUDE.md](CLAUDE.md); every closed one has a report in
[docs/MILESTONES.md](docs/MILESTONES.md). CI runs the commands above, the
Godot binding's lint and the release property tests on every push.
