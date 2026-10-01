//! refinery-scenarios: plant-definition file format (TOML) and the
//! EngineBuilder that instantiates an Engine from it — including the
//! fidelity selection (which solver implementations to plug in).
//!
//! The scenario file is the ONLY place fidelity is chosen. See
//! scenarios/tank_pump_valve.toml for the reference example of the format.
//!
//! Unit conventions in scenario files (converted to SI on load):
//! - pressures in bar (absolute), temperatures in °C, lengths in m,
//!   volumes in m³, flow coefficients as customary metric Kv (m³/h at 1 bar)
//!   — human-friendly at the boundary, SI inside, per CLAUDE.md rule 4.

mod build;
mod schema;
mod validate;

// The crate's API is unchanged by the split into three modules: every type a
// frontend or a test names is re-exported here, so `refinery_scenarios::load_str`
// and `refinery_scenarios::NodeDef` resolve exactly as they did when this file
// held all of it.
pub use build::build_engine;
pub use schema::{
    load_str, ActuatorDef, CascadeDef, ComponentDef, ControlDef, DrawDef, ExchangerDef, Fidelity,
    LoopActuatorDef, MeasurementDef, Meta, NodeDef, PipeDef, ScenarioFile, Simulation,
    TripActionDef, TripDef,
};

#[cfg(test)]
mod tests {
    /// The reference scenario file must always parse against the schema.
    #[test]
    fn reference_scenario_parses() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).expect("tank_pump_valve.toml must parse");
        assert_eq!(s.meta.name, "tank_pump_valve");
        assert_eq!(s.nodes.len(), 4);
        assert_eq!(s.pipes.len(), 3);
        assert_eq!(s.fidelity.flow, "newton");
        // IndexMap preserves file order → deterministic node ids.
        assert_eq!(s.nodes.get_index(0).unwrap().0, "supply_tank");
    }

    /// `ambient_exchange_ua_w_per_k` is optional and defaults to a perfectly insulated
    /// tank. This is what keeps every scenario written before the field existed
    /// bit-identical — `tank_pump_valve.toml` does not mention it, and must
    /// still load a tank with UA = 0 rather than failing to parse or picking up
    /// some other number.
    #[test]
    fn a_tank_without_an_ambient_ua_is_perfectly_insulated() {
        use refinery_core::graph::NodeKind;

        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).unwrap();
        let engine = super::build_engine(&s).expect("reference plant must build");
        let id = engine.graph.find_node("supply_tank").unwrap();
        match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => assert_eq!(
                t.ambient_ua.value(),
                0.0,
                "a tank whose file omits ambient_exchange_ua_w_per_k must be insulated"
            ),
            _ => panic!("supply_tank must be a tank"),
        }
    }

    /// The pipe field is optional too, and it matters more than the tank's: a
    /// pipe is the one body EVERY scenario has, so a nonzero default would move
    /// the answer of every file ever written rather than only those with tanks.
    #[test]
    fn a_pipe_without_an_ambient_ua_is_perfectly_insulated() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).unwrap();
        let engine = super::build_engine(&s).expect("reference plant must build");
        for edge in engine.graph.edge_ids() {
            let pipe = engine.graph.pipe(edge);
            assert_eq!(
                pipe.ambient_ua.value(),
                0.0,
                "pipe '{}' omits ambient_exchange_ua_w_per_k and must be insulated",
                pipe.name
            );
        }
    }

    /// A pipe `UA` written in a file reaches the pipe unchanged, and is already
    /// SI for the same reason the tank's is.
    #[test]
    fn an_ambient_ua_reaches_the_pipe_in_watts_per_kelvin() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml").replace(
            "name = \"suction\"",
            "name = \"suction\"\nambient_exchange_ua_w_per_k = 750.0",
        );
        let s = super::load_str(&src).expect("must parse with an ambient_exchange_ua_w_per_k");
        let engine = super::build_engine(&s).expect("must build");
        let suction = engine
            .graph
            .edge_ids()
            .find(|e| engine.graph.pipe(*e).name == "suction")
            .expect("the suction pipe must exist");
        assert_eq!(
            engine.graph.pipe(suction).ambient_ua.value(),
            750.0,
            "750 W/K in the file must be 750 W/K on the pipe, unscaled"
        );
    }

    /// A negative pipe `UA` is refused at LOAD, not discovered at solve. In a
    /// pipe the sign lands in an exponent, so `UA < 0` turns the decay toward
    /// ambient into growth away from it — a plant that diverges rather than one
    /// that merely reports a wrong number.
    #[test]
    fn a_negative_pipe_ambient_ua_is_refused() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml").replace(
            "name = \"suction\"",
            "name = \"suction\"\nambient_exchange_ua_w_per_k = -1.0",
        );
        let file = super::load_str(&src).expect("must parse");
        match super::build_engine(&file) {
            Ok(_) => panic!("a negative pipe ambient_exchange_ua_w_per_k must not build"),
            Err(e) => {
                let message = e.to_string();
                assert!(
                    matches!(e, refinery_core::error::SimError::Scenario(_))
                        && message.contains("suction")
                        && message.contains("conductance"),
                    "the error must name the pipe and say why, got: {message}"
                );
            }
        }
    }

    /// A `UA` written in a file reaches the tank unchanged. Unlike every other
    /// quantity in the format it is ALREADY SI — there is no customary unit for
    /// it worth converting from — so what this pins is the absence of a
    /// conversion: bar→Pa and °C→K next door make a stray factor here the
    /// natural mistake.
    #[test]
    fn an_ambient_ua_reaches_the_tank_in_watts_per_kelvin() {
        use refinery_core::graph::NodeKind;

        let src = include_str!("../../../scenarios/tank_pump_valve.toml").replace(
            "[nodes.supply_tank]",
            "[nodes.supply_tank]\nambient_exchange_ua_w_per_k = 500.0",
        );
        let s = super::load_str(&src).expect("must parse with an ambient_exchange_ua_w_per_k");
        let engine = super::build_engine(&s).expect("must build");
        let id = engine.graph.find_node("supply_tank").unwrap();
        match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => assert_eq!(
                t.ambient_ua.value(),
                500.0,
                "500 W/K in the file must be 500 W/K in the tank, unscaled"
            ),
            _ => panic!("supply_tank must be a tank"),
        }
    }

    /// A negative `UA` is refused at load. It is a CONDUCTANCE, not a signed
    /// rate: direction already comes from `(T_ambient − T_tank)`, so a negative
    /// value does not mean "loses heat" — it inverts the driving force into
    /// positive feedback, warming a hot tank further. That plant RUNS and
    /// reports finite temperatures the whole way, which is why it has to be
    /// stopped at the file rather than caught downstream.
    #[test]
    fn a_negative_ambient_ua_is_refused() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let mut file = super::load_str(src).expect("must parse");
        match file.nodes.get_mut("supply_tank").expect("a supply_tank") {
            super::NodeDef::Tank {
                ambient_exchange_ua_w_per_k,
                ..
            } => *ambient_exchange_ua_w_per_k = -1.0,
            other => panic!("supply_tank must be a tank, got {other:?}"),
        }
        // `Engine` is not `Debug`, so unwrap the Result by hand.
        match super::build_engine(&file) {
            Ok(_) => panic!("a negative ambient_exchange_ua_w_per_k must not build"),
            Err(e) => {
                let message = e.to_string();
                assert!(
                    matches!(e, refinery_core::error::SimError::Scenario(_))
                        && message.contains("supply_tank")
                        && message.contains("conductance"),
                    "must fail as a Scenario error naming the tank and explaining \
                     why, got: {message}"
                );
            }
        }
    }

    /// **The retired spelling is refused BY NAME on both a tank and a pipe, and
    /// the reason it is a refusal at all is that nothing else would notice**
    /// (M15.1, docs/DESIGN.md §17 fork 4).
    ///
    /// Neither `NodeDef` nor `PipeDef` carries `deny_unknown_fields` — only
    /// `ControlDef` does, and its doc comment says so — so a file still saying
    /// `ambient_ua_w_per_k` would otherwise parse, drop the key, and run with
    /// `UA = 0`. On `crude_column_recovery.toml` that is a condenser silently
    /// switched off: the plant loads, ticks all 6 000 ticks and recovers 20.7%
    /// where its author asked for 42.4%. The counterfactual is shown rather
    /// than described — the second half of this test deserializes the same
    /// document with the tombstone bypassed and checks that the tank really
    /// does come out with no `UA`.
    #[test]
    fn the_retired_ambient_ua_spelling_is_refused_by_name() {
        use refinery_core::graph::NodeKind;

        for (label, needle, replacement) in [
            (
                "tank",
                "supply_tank",
                (
                    "[nodes.supply_tank]",
                    "[nodes.supply_tank]\nambient_ua_w_per_k = 500.0",
                ),
            ),
            (
                "pipe",
                "suction",
                (
                    "name = \"suction\"",
                    "name = \"suction\"\nambient_ua_w_per_k = 750.0",
                ),
            ),
        ] {
            let src = include_str!("../../../scenarios/tank_pump_valve.toml")
                .replace(replacement.0, replacement.1);
            let file = super::load_str(&src)
                .unwrap_or_else(|e| panic!("{label}: the old key must still PARSE: {e}"));
            match super::build_engine(&file) {
                Ok(_) => panic!("{label}: a file using the retired spelling must not build"),
                Err(e) => {
                    let message = e.to_string();
                    assert!(
                        message.contains(needle)
                            && message.contains("ambient_exchange_ua_w_per_k")
                            && message.contains("renamed"),
                        "{label}: the refusal must name the element and the new key, \
                         got: {message}"
                    );
                }
            }
        }

        // The counterfactual: with the tombstone gone this is what the file
        // would have meant. Written with the NEW key absent and the old one
        // present under a name the schema does not know, which is exactly the
        // state a bare rename would have left.
        let src = include_str!("../../../scenarios/tank_pump_valve.toml").replace(
            "[nodes.supply_tank]",
            "[nodes.supply_tank]\nambient_ua_from_some_older_format_w_per_k = 500.0",
        );
        let file = super::load_str(&src).expect("an unknown key on a tank still parses");
        let engine = super::build_engine(&file).expect("and still builds");
        let id = engine.graph.find_node("supply_tank").unwrap();
        match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => assert_eq!(
                t.ambient_ua.value(),
                0.0,
                "an unrecognised key on a tank is DROPPED, not refused — which is why \
                 the retired spelling needs a tombstone of its own"
            ),
            _ => panic!("supply_tank must be a tank"),
        }
    }

    /// The reference plant must build into a wired graph and actually run:
    /// this is the end-to-end proof that node/pipe instantiation, unit
    /// conversions, and the fold-at-source device direction are all correct.
    #[test]
    fn build_engine_wires_and_runs_the_reference_plant() {
        use refinery_core::graph::NodeKind;

        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).unwrap();
        let mut engine = super::build_engine(&s).expect("reference plant must build");
        // Four declared nodes and three declared pipes, plus what the loader
        // adds for the brim (M23, docs/DESIGN.md §27 fork 2): one overflow edge
        // per tank, to an `overflow_atmosphere` built because this plant
        // declares no atmosphere of its own.
        assert_eq!(engine.graph.node_count(), 4 + 1);
        assert_eq!(engine.graph.edge_count(), 3 + 2);
        assert!(engine.graph.find_node("overflow_atmosphere").is_some());

        let receiving_mass = |e: &refinery_core::engine::Engine| {
            let id = e.graph.find_node("receiving_tank").unwrap();
            match &e.graph.node(id).kind {
                NodeKind::Tank(t) => t.mass.value(),
                _ => panic!("receiving_tank must be a tank"),
            }
        };
        let before = receiving_mass(&engine);

        // ~50 ticks exercises convergence + finiteness every tick (the 1000-tick
        // mass-balance run is a separate M1 acceptance gate). The pump drives
        // supply→receiving unambiguously (supply bottom ≈180 kPa, receiving
        // ≈111 kPa, pump adds up to ρg·40 ≈392 kPa against the 5 m ≈49 kPa
        // fill-line rise), so the receiving tank must gain mass.
        for _ in 0..50 {
            engine
                .tick()
                .expect("every tick must converge with no non-finite state");
        }
        let after = receiving_mass(&engine);
        assert!(
            after > before,
            "receiving tank must fill: before={before}, after={after}"
        );
    }

    #[test]
    fn unknown_solver_is_a_clear_error() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let mut s = super::load_str(src).unwrap();
        s.fidelity.flow = "quantum".into();
        let err = match super::build_engine(&s) {
            Err(e) => e,
            Ok(_) => panic!("expected an error for unknown solver"),
        };
        assert!(err.to_string().contains("quantum"));
    }
}
