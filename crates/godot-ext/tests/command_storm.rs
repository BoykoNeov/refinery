//! Random operator commands at every shipped plant: the engine never panics
//! and never hands a frontend a snapshot carrying NaN (CLAUDE.md rule 5).
//!
//! Every other gate sends the commands a story needs. A player sends anything:
//! a valve on a node that is a tank, a duty of 1e300 W, a puncture wider than
//! the pipe, a trip reset on a plant without trips. Each of those may be
//! refused, and a tick after an accepted one may fail with an `Err` — both
//! are the engine working. What may not happen is a panic (the game process
//! dies) or a tick that reports success and a snapshot with NaN in it (the
//! screen draws garbage). Found clean on all 71 plants when written
//! (2026-11-12): 108 failed ticks, no panic, no NaN.
//!
//! What it does NOT gate, measured the same day and left to
//! `docs/plans/failed-tick-recovery.md`: a failed tick is not recoverable.
//! None of the 108 failures ticked again afterwards, and in 15 the failed
//! retries still changed the snapshot.
//!
//! The commands are drawn from a fixed-seed generator (no dependency, the
//! same sequence on every machine), over ids valid for the plant and a pool
//! of values from the ordinary to the absurd. Split in four by plant so the
//! harness runs the quarters in parallel; `slow_` because each is seconds.

use refinery_godot_ext::bridge::Session;
use std::panic::{catch_unwind, AssertUnwindSafe};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        // xorshift64*
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: u64) -> u64 {
        (self.next() >> 11) % n.max(1)
    }
    fn pick<'a, T>(&mut self, xs: &'a [T]) -> &'a T {
        &xs[self.below(xs.len() as u64) as usize]
    }
}

const VALUES: &[f64] = &[
    0.0, 1.0, 0.5, 0.01, 0.99, -1.0, 2.0, 1e-12, 1e-6, 10.0, 100.0, 273.15, 300.0, 400.0, 600.0,
    900.0, 2000.0, 1e4, 1e5, 2e5, 1e6, 5e6, 2e7, 1e8, 1e9, 1e12, 1e300, -1e6, 5.0, 20.0,
];

fn command(rng: &mut Rng, nodes: u64, edges: u64) -> String {
    let n = rng.below(nodes);
    let e = rng.below(edges);
    let v = *rng.pick(VALUES);
    let i = rng.below(4);
    let b = rng.below(2) == 0;
    match rng.below(24) {
        0 => format!(r#"{{"cmd":"set_valve_opening","node":{n},"opening":{v:e}}}"#),
        1 => format!(r#"{{"cmd":"set_pump_on","node":{n},"on":{b}}}"#),
        2 => format!(r#"{{"cmd":"puncture_pipe","edge":{e},"area":{v:e}}}"#),
        3 => format!(r#"{{"cmd":"set_heat_input","node":{n},"power":{v:e}}}"#),
        4 => format!(r#"{{"cmd":"set_furnace_duty","node":{n},"duty":{v:e}}}"#),
        5 => format!(r#"{{"cmd":"set_cooler_duty","node":{n},"duty":{v:e}}}"#),
        6 => format!(r#"{{"cmd":"set_cooling_water_flow","node":{n},"flow":{v:e}}}"#),
        7 => format!(
            r#"{{"cmd":"set_controller_mode","loop_id":{i},"mode":"{}"}}"#,
            if b { "auto" } else { "manual" }
        ),
        8 => {
            let tag = *rng.pick(&[
                ("level", "m"),
                ("pressure", "pa"),
                ("temperature", "k"),
                ("flow", "kg_s"),
            ]);
            format!(
                r#"{{"cmd":"set_setpoint","loop_id":{i},"value":{{"{}":{{"{}":{v:e}}}}}}}"#,
                tag.0, tag.1
            )
        }
        9 => format!(r#"{{"cmd":"replace_tubes","node":{n}}}"#),
        10 => format!(r#"{{"cmd":"repair_bed_wall","node":{n}}}"#),
        11 => format!(r#"{{"cmd":"vent_pump","node":{n}}}"#),
        12 => format!(r#"{{"cmd":"reset_trip","trip_id":{i}}}"#),
        13 => format!(r#"{{"cmd":"manual_trip","trip_id":{i}}}"#),
        14 => format!(r#"{{"cmd":"set_reservoir_pressure","node":{n},"pressure":{v:e}}}"#),
        15 => format!(r#"{{"cmd":"set_source_temperature","node":{n},"temperature":{v:e}}}"#),
        16 => format!(r#"{{"cmd":"set_reflux_ratio","node":{n},"reflux_ratio":{v:e}}}"#),
        17 => {
            format!(r#"{{"cmd":"set_pumparound_duty","node":{n},"pumparound":{i},"duty":{v:e}}}"#)
        }
        18 => format!(r#"{{"cmd":"set_loop_flow","node":{n},"pumparound_loop":{i},"flow":{v:e}}}"#),
        19 => format!(
            r#"{{"cmd":"set_loop_return_temperature","node":{n},"pumparound_loop":{i},"temperature":{v:e}}}"#
        ),
        20 => {
            if b {
                format!(r#"{{"cmd":"set_reboiler_duty","node":{n},"duty":{v:e}}}"#)
            } else {
                format!(r#"{{"cmd":"set_reboiler_duty","node":{n},"duty":null}}"#)
            }
        }
        21 => format!(r#"{{"cmd":"set_total_reflux","node":{n},"on":{b}}}"#),
        22 => format!(r#"{{"cmd":"set_slop_drain","node":{n},"open":{b}}}"#),
        _ => format!(
            r#"{{"cmd":"set_valve_opening","node":{n},"opening":{:e}}}"#,
            (rng.below(101) as f64) / 100.0
        ),
    }
}

/// One plant's storm. `None` when nothing forbidden happened.
fn storm(name: &str, src: &str, seed: u64) -> Option<String> {
    let mut accepted: Vec<String> = Vec::new();
    let outcome = catch_unwind(AssertUnwindSafe(|| {
        let mut session = Session::default();
        assert_eq!(session.load_scenario(src), "null", "{name} loads");
        let count = |json: String| serde_json::from_str::<Vec<String>>(&json).unwrap().len() as u64;
        let nodes = count(session.node_names_json());
        let edges = count(session.edge_names_json());
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15 ^ (seed * 7919));
        for _ in 0..20 {
            assert_eq!(session.tick(), "null", "{name} runs before any command");
        }
        for _round in 0..60 {
            for _ in 0..(1 + rng.below(3)) {
                let text = command(&mut rng, nodes, edges);
                if session.apply_command_json(&text) == "null" {
                    accepted.push(format!("t{} {text}", session.tick_index()));
                }
            }
            for _ in 0..15 {
                if session.tick() != "null" {
                    // A refusal is the engine working; the storm on this
                    // plant ends here (a failed tick does not recover).
                    return None;
                }
                let json = session.snapshot_json();
                // NaN serializes as `null`, which is not an `f64`.
                if serde_json::from_str::<refinery_core::snapshot::Snapshot>(&json).is_err() {
                    return Some(format!(
                        "a NaN in the snapshot of tick {}",
                        session.tick_index()
                    ));
                }
            }
        }
        None
    }));
    let last: Vec<&String> = accepted.iter().rev().take(4).rev().collect();
    match outcome {
        Ok(None) => None,
        Ok(Some(what)) => Some(format!(
            "{name} (seed {seed}): {what}; last accepted: {last:?}"
        )),
        Err(panic) => {
            let message = panic
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_default();
            Some(format!(
                "{name} (seed {seed}): PANIC {message}; last accepted: {last:?}"
            ))
        }
    }
}

/// Every fourth shipped plant, starting at `quarter`.
fn storm_quarter(quarter: usize) {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().is_some_and(|e| e == "toml"))
        .collect();
    files.sort();
    assert!(files.len() >= 71, "the shipped plants are found");
    let mut findings = Vec::new();
    for file in files.iter().skip(quarter).step_by(4) {
        let name = file.file_stem().unwrap().to_string_lossy().to_string();
        let src = std::fs::read_to_string(file).unwrap();
        for seed in 1..=2 {
            findings.extend(storm(&name, &src, seed));
        }
    }
    assert!(
        findings.is_empty(),
        "{}",
        findings.join(
            "
"
        )
    );
}

#[test]
fn slow_random_commands_never_panic_or_emit_nan_quarter_0() {
    storm_quarter(0);
}

#[test]
fn slow_random_commands_never_panic_or_emit_nan_quarter_1() {
    storm_quarter(1);
}

#[test]
fn slow_random_commands_never_panic_or_emit_nan_quarter_2() {
    storm_quarter(2);
}

#[test]
fn slow_random_commands_never_panic_or_emit_nan_quarter_3() {
    storm_quarter(3);
}
