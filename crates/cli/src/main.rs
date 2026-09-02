//! Headless runner: the experimentation/regression frontend.
//!
//!   refinery run scenarios/tank_pump_valve.toml --ticks 1000
//!   refinery run scenarios/tank_pump_valve.toml --ticks 1000 \
//!       --solver simple --snapshot-every 10 --out run.jsonl
//!
//! Output: JSON-lines, one Snapshot per line — trivially consumable by
//! Python/pandas, jq, or a future dashboard.
//!
//!   refinery corpus scenarios --ticks 6000
//!   refinery corpus scenarios --ticks 6000 --solver simple
//!   refinery corpus scenarios --ticks 6000 --out before.json
//!   refinery corpus scenarios --ticks 6000 --baseline before.json
//!
//! `corpus` is the measurement every solver slice since M9.0 has made by hand:
//! run every shipped scenario for N ticks and report, per plant, the worst
//! solver iteration count in any tick, the total, the wall time of the tick
//! loop alone, and a fingerprint of every snapshot it emitted. Two runs whose
//! fingerprints agree ran byte-identical; `--baseline` says which plants moved
//! and exits nonzero if any did. A plant that fails to load or fails a tick is
//! an error of the run itself: it is reported in the status column AND exits
//! nonzero, with or without a baseline, because a corpus that reports a
//! divergence and then exits 0 is a check that cannot fail. The table is
//! printed as markdown because that is the shape DESIGN.md and ROADMAP.md
//! record it in.

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

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
        scenario: PathBuf,
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
        out: Option<PathBuf>,
    },
    /// Run every scenario in a directory (or the files given) and report the
    /// solver's worst and total iterations, the tick-loop wall time, and a
    /// fingerprint over every snapshot, one row per plant.
    Corpus {
        /// Scenario files, or directories to scan for `*.toml`.
        #[arg(required = true)]
        paths: Vec<PathBuf>,
        #[arg(long, default_value_t = 6000)]
        ticks: u64,
        /// Override every scenario's flow solver ("newton" | "simple").
        #[arg(long)]
        solver: Option<String>,
        /// Write the rows as JSON here, for a later `--baseline`.
        #[arg(long)]
        out: Option<PathBuf>,
        /// A JSON file written by an earlier `--out`; each row is compared to
        /// its namesake and the process exits nonzero if any fingerprint moved.
        #[arg(long)]
        baseline: Option<PathBuf>,
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
        } => run(&scenario, ticks, solver.as_deref(), snapshot_every, out),
        Cmd::Corpus {
            paths,
            ticks,
            solver,
            out,
            baseline,
        } => corpus(&paths, ticks, solver.as_deref(), out, baseline),
    }
}

/// Load a scenario, apply the optional flow-solver override, and build the
/// engine. Returns the flow fidelity the engine was actually built with — the
/// file's own, its default when the key is omitted, or the override.
fn load(scenario: &Path, solver: Option<&str>) -> Result<(refinery_core::engine::Engine, String)> {
    let src = std::fs::read_to_string(scenario)
        .with_context(|| format!("reading {}", scenario.display()))?;
    let mut file = refinery_scenarios::load_str(&src)?;
    if let Some(s) = solver {
        file.fidelity.flow = s.to_string();
    }
    let engine = refinery_scenarios::build_engine(&file)?;
    Ok((engine, file.fidelity.flow))
}

fn run(
    scenario: &Path,
    ticks: u64,
    solver: Option<&str>,
    snapshot_every: u64,
    out: Option<PathBuf>,
) -> Result<()> {
    let (mut engine, _) = load(scenario, solver)?;

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

/// One plant's row of the corpus table.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct CorpusRow {
    /// The scenario file's stem, e.g. `tank_pump_valve`.
    name: String,
    /// The flow solver the run actually used, after any override.
    flow: String,
    /// Ticks completed before the run ended (equals the request on `ok`).
    ticks: u64,
    /// `ok`, or `err @ tick N: message`.
    status: String,
    /// Largest `solver.iterations` reported by any tick.
    worst_iterations: u32,
    /// Sum of `solver.iterations` over every tick.
    total_iterations: u64,
    /// Wall time of the tick loop alone, snapshots and hashing excluded.
    tick_wall_ms: f64,
    /// FNV-1a 64 over the JSON of every tick's snapshot, in tick order.
    /// Same scenario, same code, same run length ⇒ same fingerprint; this is
    /// the "runs byte-identical" claim as a number.
    fingerprint: String,
}

/// The status of a row that ran every tick it was asked for. Anything else is
/// a message, and `CorpusRow::errored` is the only reader that needs to tell
/// the two apart.
const STATUS_OK: &str = "ok";

impl CorpusRow {
    /// Did this plant fail to load, or fail a tick?
    fn errored(&self) -> bool {
        self.status != STATUS_OK
    }
}

/// A row's standing against a baseline run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    /// No plant of this name in the baseline.
    New,
    /// Same solver, same fingerprint: this plant runs byte-identical.
    Identical,
    /// Some number changed — or the run used a different solver, which makes
    /// the fingerprints incomparable rather than equal.
    Moved,
}

/// Grade one row against the baseline rows.
///
/// Rows are matched by scenario NAME alone, deliberately. `flow` cannot be part
/// of the key: it is only known after a successful load, so a plant that fails
/// to load carries the `(file)` placeholder, would miss its baseline row, and
/// would be reported as `new` — the one verdict that is not counted as moved.
/// A solver that differs from the baseline's is a real difference, so it is
/// compared as a value instead of silently deciding which rows are comparable.
fn verdict(row: &CorpusRow, baseline: &[CorpusRow]) -> Verdict {
    match baseline.iter().find(|b| b.name == row.name) {
        None => Verdict::New,
        Some(b) if b.flow == row.flow && b.fingerprint == row.fingerprint => Verdict::Identical,
        Some(_) => Verdict::Moved,
    }
}

fn corpus(
    paths: &[PathBuf],
    ticks: u64,
    solver: Option<&str>,
    out: Option<PathBuf>,
    baseline: Option<PathBuf>,
) -> Result<()> {
    let files = collect_scenarios(paths)?;
    anyhow::ensure!(!files.is_empty(), "no scenario files found");

    let baseline: Option<Vec<CorpusRow>> = match baseline {
        Some(p) => {
            let src =
                std::fs::read_to_string(&p).with_context(|| format!("reading {}", p.display()))?;
            Some(serde_json::from_str(&src).with_context(|| format!("parsing {}", p.display()))?)
        }
        None => None,
    };

    let mut rows = Vec::with_capacity(files.len());
    for file in &files {
        rows.push(run_one(file, ticks, solver));
    }

    print_table(&rows, baseline.as_deref());

    if let Some(p) = out {
        std::fs::write(&p, serde_json::to_string_pretty(&rows)?)
            .with_context(|| format!("writing {}", p.display()))?;
    }

    // A plant that did not finish is a failure of the run whether or not a
    // baseline was asked for, and it is checked FIRST: an errored row's
    // fingerprint differs too, so grading it against a baseline would report
    // "moved" for a plant whose real news is in the status column.
    let failed: Vec<&str> = rows
        .iter()
        .filter(|r| r.errored())
        .map(|r| r.name.as_str())
        .collect();
    anyhow::ensure!(
        failed.is_empty(),
        "{} of {} plants did not finish {} ticks: {} (see the status column)",
        failed.len(),
        rows.len(),
        ticks,
        failed.join(", ")
    );

    if let Some(base) = baseline {
        let moved: Vec<&str> = rows
            .iter()
            .filter(|r| verdict(r, &base) == Verdict::Moved)
            .map(|r| r.name.as_str())
            .collect();
        anyhow::ensure!(
            moved.is_empty(),
            "{} of {} plants moved against the baseline: {}",
            moved.len(),
            rows.len(),
            moved.join(", ")
        );
    }
    Ok(())
}

fn collect_scenarios(paths: &[PathBuf]) -> Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    for p in paths {
        if p.is_dir() {
            for entry in std::fs::read_dir(p).with_context(|| format!("listing {}", p.display()))? {
                let path = entry?.path();
                if path.extension().is_some_and(|e| e == "toml") {
                    files.push(path);
                }
            }
        } else {
            files.push(p.clone());
        }
    }
    // Directory order is filesystem order; the table is sorted by name so two
    // runs on two machines line up row for row.
    files.sort();
    files.dedup();
    Ok(files)
}

fn run_one(file: &Path, ticks: u64, solver: Option<&str>) -> CorpusRow {
    let name = file
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| file.display().to_string());
    let mut row = CorpusRow {
        name,
        flow: solver.unwrap_or("(file)").to_string(),
        ticks: 0,
        status: STATUS_OK.into(),
        worst_iterations: 0,
        total_iterations: 0,
        tick_wall_ms: 0.0,
        fingerprint: String::new(),
    };

    let mut engine = match load(file, solver) {
        Ok((engine, flow)) => {
            row.flow = flow;
            engine
        }
        Err(e) => {
            row.status = format!("err @ load: {e:#}");
            return row;
        }
    };

    let mut hash = Fnv1a64::new();
    let mut wall = 0.0f64;
    for t in 1..=ticks {
        let started = Instant::now();
        let result = engine.tick();
        wall += started.elapsed().as_secs_f64() * 1e3;
        if let Err(e) = result {
            row.status = format!("err @ tick {t}: {e}");
            break;
        }
        row.ticks = t;
        let snapshot = engine.snapshot();
        row.worst_iterations = row.worst_iterations.max(snapshot.solver.iterations);
        row.total_iterations += u64::from(snapshot.solver.iterations);
        match serde_json::to_vec(&snapshot) {
            Ok(bytes) => {
                hash.write(&bytes);
                hash.write(b"\n");
            }
            Err(e) => {
                row.status = format!("err @ tick {t}: snapshot did not serialize: {e}");
                break;
            }
        }
    }
    row.tick_wall_ms = wall;
    row.fingerprint = format!("{:016x}", hash.finish());
    row
}

fn print_table(rows: &[CorpusRow], baseline: Option<&[CorpusRow]>) {
    let mut out = std::io::stdout().lock();
    let mut line = String::from(
        "| scenario | flow | ticks | worst iter | total iter | tick wall ms | fingerprint |",
    );
    let mut rule = String::from("|---|---|---:|---:|---:|---:|---|");
    if baseline.is_some() {
        line.push_str(" vs baseline |");
        rule.push_str("---|");
    }
    line.push_str(" status |");
    rule.push_str("---|");
    let _ = writeln!(out, "{line}\n{rule}");
    for r in rows {
        let mut cells = format!(
            "| `{}` | {} | {} | {} | {} | {:.1} | `{}` |",
            r.name,
            r.flow,
            r.ticks,
            r.worst_iterations,
            r.total_iterations,
            r.tick_wall_ms,
            r.fingerprint
        );
        if let Some(base) = baseline {
            let label = match verdict(r, base) {
                Verdict::New => "new",
                Verdict::Identical => "identical",
                Verdict::Moved => "**moved**",
            };
            cells.push_str(&format!(" {label} |"));
        }
        cells.push_str(&format!(" {} |", r.status));
        let _ = writeln!(out, "{cells}");
    }
}

/// FNV-1a, 64-bit. Not a cryptographic hash and not meant as one: it is a
/// fingerprint for "did this byte stream change", chosen because it is twenty
/// lines with no dependency and is identical on every platform.
struct Fnv1a64(u64);

impl Fnv1a64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;

    fn new() -> Self {
        Self(Self::OFFSET)
    }

    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(Self::PRIME);
        }
    }

    fn finish(&self) -> u64 {
        self.0
    }
}
