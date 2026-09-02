# Refinery Simulator

Headless, deterministic oil-refinery simulation engine in Rust with swappable
fidelity (game ↔ research), consumed by a Godot 4 game, a CLI batch runner,
and future dashboards.

- **Start here:** [CLAUDE.md](CLAUDE.md) — project rules, conventions, and the
  current milestone's state
- **Architecture & physics:** [docs/DESIGN.md](docs/DESIGN.md) — every design
  note, written before its slice was built and corrected after
- **Milestones:** [docs/ROADMAP.md](docs/ROADMAP.md) — M1–M8 closed, M9 open
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
change, compare after, and the command exits nonzero if any plant moved.

Status: M1–M8 closed. **M9 (solver robustness) is open**; three slices have
landed and the next is scoped from a measurement, not a list — see the M9.3
probe in the roadmap. CI runs the four commands above plus the Godot binding's
lint on every push.
