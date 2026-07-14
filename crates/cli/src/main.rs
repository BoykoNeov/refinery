//! Headless runner: the experimentation/regression frontend.
//!
//!   refinery run scenarios/tank_pump_valve.toml --ticks 1000
//!   refinery run scenarios/tank_pump_valve.toml --ticks 1000 \
//!       --solver simple --snapshot-every 10 --out run.jsonl
//!
//! Output: JSON-lines, one Snapshot per line — trivially consumable by
//! Python/pandas, jq, or a future dashboard.

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use std::io::Write;

#[derive(Parser)]
#[command(name = "refinery", about = "Headless refinery simulation runner")]
struct Cli {
    #[command(subcommand)]
    command: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Load a scenario and run it for a number of ticks.
    Run {
        scenario: std::path::PathBuf,
        #[arg(long, default_value_t = 1000)]
        ticks: u64,
        /// Override the scenario's flow solver ("newton" | "simple").
        #[arg(long)]
        solver: Option<String>,
        /// Emit a snapshot every N ticks (0 = only final).
        #[arg(long, default_value_t = 10)]
        snapshot_every: u64,
        /// Output file (JSON lines); stdout if omitted.
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Cmd::Run {
            scenario,
            ticks,
            solver,
            snapshot_every,
            out,
        } => {
            let src = std::fs::read_to_string(&scenario)
                .with_context(|| format!("reading {}", scenario.display()))?;
            let mut file = refinery_scenarios::load_str(&src)?;
            if let Some(s) = solver {
                file.fidelity.flow = s;
            }
            let mut engine = refinery_scenarios::build_engine(&file)?;

            let mut sink: Box<dyn Write> = match out {
                Some(p) => Box::new(std::fs::File::create(p)?),
                None => Box::new(std::io::stdout().lock()),
            };

            for t in 1..=ticks {
                engine.tick().with_context(|| format!("tick {t} failed"))?;
                let emit = t == ticks || (snapshot_every > 0 && t % snapshot_every == 0);
                if emit {
                    serde_json::to_writer(&mut sink, &engine.snapshot())?;
                    sink.write_all(b"\n")?;
                }
            }
            Ok(())
        }
    }
}
