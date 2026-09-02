//! The corpus subcommand's EXIT STATUS, which is the whole of its value as an
//! automated check: `.github/workflows/ci.yml` runs it on every push and reads
//! nothing but the exit code.
//!
//! These are process-level tests because that is where the defect lived. The
//! first version of `corpus` recorded a diverging plant in the status column of
//! its markdown table, printed the table, and returned `Ok(())` — a check whose
//! comment said it "fails on any Err" and which could not fail. Nothing inside
//! the process observes the difference; only a caller reading the exit code
//! does, so only a caller reading the exit code can gate it.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

/// A plant that parses but is not a scenario: the loader refuses it, so the row
/// ends `err @ load` before a single tick runs. A tick-time divergence would
/// exercise the other arm, but no shipped plant diverges on demand and a made
/// up one would be a second thing to maintain.
const NOT_A_SCENARIO: &str = "definitely = \"not a plant\"\n";

/// The reference plant, embedded so the test does not depend on the process's
/// working directory.
const GOOD_SCENARIO: &str = include_str!("../../../scenarios/tank_pump_valve.toml");

/// A scratch directory of scenario files, removed when the test ends.
struct Corpus(PathBuf);

impl Corpus {
    fn new(tag: &str) -> Self {
        let dir =
            std::env::temp_dir().join(format!("refinery-corpus-{}-{tag}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        Self(dir)
    }

    fn write(&self, stem: &str, body: &str) {
        std::fs::write(self.0.join(format!("{stem}.toml")), body).expect("writing a scenario");
    }

    fn path(&self) -> &Path {
        &self.0
    }

    /// Run `refinery corpus` over this directory. `cargo` builds the binary for
    /// an integration test and hands us its path.
    fn run(&self, extra: &[&str]) -> Output {
        Command::new(env!("CARGO_BIN_EXE_refinery"))
            .arg("corpus")
            .arg(self.path())
            .args(["--ticks", "5"])
            .args(extra)
            .output()
            .expect("the corpus binary must run")
    }
}

impl Drop for Corpus {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// The control, and it has to come first: without it "exits nonzero" is passed
/// by a corpus that fails on everything.
#[test]
fn a_corpus_of_healthy_plants_exits_zero() {
    let corpus = Corpus::new("healthy");
    corpus.write("tank_pump_valve", GOOD_SCENARIO);

    let out = corpus.run(&[]);
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        out.status.success(),
        "a plant that runs must exit 0, got {:?}\n{stdout}\n{}",
        out.status.code(),
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        stdout.contains("tank_pump_valve"),
        "the table must name the plant it ran: {stdout}"
    );
}

/// The defect itself. A plant that cannot run must fail the run — not merely be
/// mentioned in a column of a table nothing reads.
#[test]
fn a_plant_that_cannot_load_fails_the_corpus_run() {
    let corpus = Corpus::new("broken");
    corpus.write("tank_pump_valve", GOOD_SCENARIO);
    corpus.write("broken_plant", NOT_A_SCENARIO);

    let out = corpus.run(&[]);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        !out.status.success(),
        "a plant that fails to load must exit nonzero, got {:?}\n{}",
        out.status.code(),
        String::from_utf8_lossy(&out.stdout)
    );
    assert!(
        stderr.contains("broken_plant"),
        "the error must name the plant that failed, got: {stderr}"
    );
    assert!(
        String::from_utf8_lossy(&out.stdout).contains("err @ load"),
        "the table must still be printed, so the reason is visible beside the \
         exit code"
    );
}

/// The narrower half of the same defect, and the reason baseline rows are
/// matched by NAME alone. A row's `flow` is the placeholder `(file)` until the
/// plant loads, so a load failure used to miss its baseline row on a
/// `name + flow` key, be graded `new`, and escape the "did anything move?"
/// filter — the plant most obviously not identical was the one exempt from the
/// comparison.
#[test]
fn a_plant_that_breaks_after_a_baseline_is_moved_not_new() {
    let corpus = Corpus::new("baseline");
    corpus.write("tank_pump_valve", GOOD_SCENARIO);

    let before = corpus.path().join("before.json");
    let out = corpus.run(&["--out", &before.display().to_string()]);
    assert!(out.status.success(), "the baseline run must succeed");

    // Same plant, same name, now unloadable.
    corpus.write("tank_pump_valve", NOT_A_SCENARIO);
    let out = corpus.run(&["--baseline", &before.display().to_string()]);
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        !out.status.success(),
        "a plant that broke since the baseline must exit nonzero"
    );
    // The exit code cannot discriminate here: the errored-plants check fires
    // first and would fail this run under the old key too. What is being tested
    // is the VERDICT, so this assertion is a string match against the table's
    // layout — reformat the verdict column and this catch is silently gone.
    assert!(
        stdout.contains("**moved**") && !stdout.contains(" new "),
        "the verdict column must read moved, not new: {stdout}"
    );
}
