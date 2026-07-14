# Refinery Simulator

Headless, deterministic oil-refinery simulation engine in Rust with swappable
fidelity (game ↔ research), consumed by a Godot 4 game, a CLI batch runner,
and future dashboards.

- **Start here:** [CLAUDE.md](CLAUDE.md) — project rules and conventions
- **Architecture & physics:** [docs/DESIGN.md](docs/DESIGN.md)
- **Milestones:** [docs/ROADMAP.md](docs/ROADMAP.md)

```sh
cargo test --workspace
cargo run -p refinery-cli -- run scenarios/tank_pump_valve.toml --ticks 1000
```

Status: scaffold. Current milestone: **M1 — flow network core**
(Newton hydraulic solver + relaxation solver + property tests).
