//! Every `[fidelity]` key refuses a value it does not implement.
//!
//! Written for M7.2, and it is a defect gate before it is a feature gate.
//! `thermo` was parsed and then **discarded** — `build_engine` hardcoded
//! `ConstantThermo` — so `thermo = "nonsense"` loaded a working plant while the
//! three keys beside it each refused an unknown value with a message listing the
//! valid ones. Nothing reached the value, so no test could have failed on it;
//! it surfaced only when M7.1 wired `separation` in alongside.
//!
//! The four arms are gated together on purpose. `column_reference.rs` calls the
//! refusal "the same contract every other `[fidelity]` string has" — a claim
//! that was false when it was written, and one that a per-key test would let
//! drift again as keys are added. Here the claim itself is the test.

use refinery_scenarios::{build_engine, load_str};

/// A minimal water-only plant, pressure-anchored so it actually builds: the
/// point is which fidelity STRING is refused, so the plant beneath it is kept as
/// small as a loadable plant can be.
fn plant(fidelity: &str) -> String {
    format!(
        r#"
[meta]
name = "fidelity_selection"

[simulation]
dt = 0.1

[fidelity]
{fidelity}

[nodes.feed]
type = "source"
pressure_bar = 5.0
temperature_c = 20.0

[nodes.storage]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 2.0
temperature_c = 20.0

[[pipes]]
name = "fill"
from = "feed"
to = "storage"
length_m = 10.0
diameter_m = 0.10
"#
    )
}

fn build(fidelity: &str) -> Result<(), String> {
    let scenario = load_str(&plant(fidelity)).map_err(|e| e.to_string())?;
    build_engine(&scenario)
        .map(|_| ())
        .map_err(|e| e.to_string())
}

/// The baseline: the same plant with every key at its shipped value builds. A
/// refusal test whose plant does not load without the bad string proves nothing
/// about the string.
#[test]
fn the_baseline_plant_builds_with_every_key_at_its_shipped_value() {
    build("flow = \"newton\"\nthermo = \"constant\"\nreactions = \"none\"\nseparation = \"cut_point\"")
        .expect("the baseline fidelity block must build");
}

/// Each key refuses an unknown value, and the message names both the bad value
/// and the valid ones — the second half is what makes the error actionable
/// rather than merely correct.
///
/// THE DEFECT THIS EXISTS FOR is the `thermo` row: before M7.2 it BUILT. The
/// other three rows are the standard it was measured against, and keeping them
/// in one test is what stops a fifth key from being added without one.
#[test]
fn every_fidelity_key_refuses_a_value_it_does_not_implement() {
    let cases = [
        ("flow", "flow = \"nonsense\"", "newton"),
        (
            "thermo",
            "flow = \"newton\"\nthermo = \"nonsense\"",
            "constant",
        ),
        (
            "reactions",
            "flow = \"newton\"\nreactions = \"nonsense\"",
            "none",
        ),
        (
            "separation",
            "flow = \"newton\"\nseparation = \"nonsense\"",
            "cut_point",
        ),
    ];
    for (key, fidelity, a_valid_value) in cases {
        let message =
            build(fidelity).expect_err(&format!("`{key} = \"nonsense\"` must be refused at load"));
        assert!(
            message.contains("nonsense"),
            "the `{key}` refusal must name the bad value, got: {message}"
        );
        assert!(
            message.contains(a_valid_value),
            "the `{key}` refusal must list what IS valid (expected '{a_valid_value}'), \
             got: {message}"
        );
    }
}

/// An omitted `thermo` key still selects the constant model — the default that
/// every file written before M7.2 means, not merely one that parses.
///
/// This is the M7.1 `separation` precedent and it is load-bearing in the same
/// way: all twelve scenario files in the repo write `thermo = "constant"`
/// explicitly, so the default is exercised by nothing else.
#[test]
fn an_omitted_thermo_key_defaults_to_the_constant_model() {
    build("flow = \"newton\"").expect("a fidelity block with only `flow` must build");
}
