//! M65 — the wet crude unit on a wetter crude (docs/DESIGN.md §70, ledger row
//! C14). `scenarios/crude_unit_wet.toml` with more water in its crude, the oil
//! scaled to make room (M63.4's recipe): the supply boils harder, the charge
//! falls, and the column's cold start runs only in a narrow band of
//! pumparounds — while the running column takes any of them (M65.0's probe,
//! `tools/steptime/prototypes/m65_0/FINDINGS.md`).

use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::Watt;
use refinery_core::Engine;

const WET: &str = include_str!("../../../scenarios/crude_unit_wet.toml");
/// The shipped crude's oil, mass fractions, in the slate's order.
const OIL: [(&str, f64); 6] = [
    ("light_naphtha", 0.07976),
    ("heavy_naphtha", 0.13958),
    ("kerosene", 0.15952),
    ("diesel", 0.1994),
    ("gas_oil", 0.13958),
    ("residue", 0.27916),
];
const PUMPAROUNDS: &str = "pumparounds = [{ stage = 4, duty_mw = 15.0 }, { stage = 6, duty_mw = 15.0 }, { stage = 7, duty_mw = 10.0 }]";
/// The pumparounds' duties as shipped [MW], in the order the file lists them.
const PUMPAROUND_MW: [f64; 3] = [15.0, 15.0, 10.0];

/// The shipped wet crude unit with `water` [mass fraction] in its crude, the
/// oil scaled to `1 − water`, and its pumparounds at `share` of the shipped.
fn wetter(water: f64, share: f64) -> String {
    let oil: f64 = OIL.iter().map(|(_, f)| f).sum();
    let parts: Vec<String> = OIL
        .iter()
        .map(|(name, f)| format!("{name} = {}", f / oil * (1.0 - water)))
        .collect();
    let composition = format!("composition = {{ {}, water = {water} }}", parts.join(", "));
    let start = WET
        .find("composition = { light_naphtha = 0.07976")
        .expect("the shipped crude");
    let end = start + WET[start..].find('}').expect("its end") + 1;
    let text = format!("{}{}{}", &WET[..start], composition, &WET[end..]);
    assert!(text.contains(PUMPAROUNDS), "the shipped pumparounds");
    text.replace(
        PUMPAROUNDS,
        &format!(
            "pumparounds = [{{ stage = 4, duty_mw = {} }}, {{ stage = 6, duty_mw = {} }}, \
             {{ stage = 7, duty_mw = {} }}]",
            PUMPAROUND_MW[0] * share,
            PUMPAROUND_MW[1] * share,
            PUMPAROUND_MW[2] * share
        ),
    )
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn run(engine: &mut Engine, ticks: u32) -> Result<Snapshot, String> {
    for tick in 1..=ticks {
        engine.tick().map_err(|e| format!("tick {tick}: {e}"))?;
    }
    Ok(engine.snapshot())
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node '{name}'"))
}

fn charge(s: &Snapshot) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == "feed_spool")
        .expect("the feed spool")
        .stream
        .mass_flow
        .value()
}

/// **A wetter crude starts on tick 1 at the shipped pumparounds** (M65.1,
/// §70.1). At 2 % water the column's cold start runs from 0.45 to 0.95 of the
/// shipped 40 MW, at 5 % from 0.50 to 0.75, at 10 % from 0.55 to 0.65 (M65.0,
/// finding 2): 40 MW refused on tick 1 until the column started again from half
/// its pumparounds and walked them back. The charge falls as the water rises
/// (76.1, 56.6, 43.7 kg/s on tick 1, finding 1).
fn starts_on_tick_1(water: f64, charge_kg_per_s: f64) {
    let mut engine = build(&wetter(water, 1.0));
    let s = run(&mut engine, 20).unwrap_or_else(|e| panic!("{water}: {e}"));
    let column = node(&s, "column");
    assert!(
        column.column_operation.is_some(),
        "{water}: the column fractionates"
    );
    assert!(
        column.column_flooded.is_none(),
        "{water}: the column is not flooded"
    );
    let first = {
        let mut engine = build(&wetter(water, 1.0));
        run(&mut engine, 1).expect("tick 1");
        charge(&engine.snapshot())
    };
    assert!(
        (first - charge_kg_per_s).abs() < 0.1,
        "{water}: the charge on tick 1 is {first} kg/s"
    );
}

#[test]
fn a_crude_2_percent_water_starts_on_tick_1() {
    starts_on_tick_1(0.02, 76.05);
}

#[test]
fn a_crude_5_percent_water_starts_on_tick_1() {
    starts_on_tick_1(0.05, 56.58);
}

#[test]
fn a_crude_10_percent_water_starts_on_tick_1() {
    starts_on_tick_1(0.10, 43.69);
}

/// **The column the retry starts is the column**: the same plant started
/// inside its cold band (`inside` of the shipped pumparounds) and stepped to
/// the shipped 40 MW on its first tick, run alongside, stands at the same
/// stage temperatures, reflux and duties 200 ticks on. Nothing upstream of the
/// column reads its pumparounds — its pressure is pinned — so the two plants
/// feed it the same at every tick.
fn the_started_column_is_the_column(water: f64, inside: f64) {
    let mut retried = build(&wetter(water, 1.0));
    let mut stepped = build(&wetter(water, inside));
    run(&mut stepped, 1).expect("inside its band, it starts");
    let column = node(&stepped.snapshot(), "column").id;
    for (i, mw) in PUMPAROUND_MW.iter().enumerate() {
        stepped
            .apply(Command::SetPumparoundDuty {
                node: column,
                pumparound: i as u32,
                duty: Watt(mw * 1e6),
            })
            .expect("the shipped duty");
    }
    let a = run(&mut retried, 200).expect("the retried plant runs");
    let b = run(&mut stepped, 199).expect("the stepped plant runs");
    let (a, b) = (node(&a, "column"), node(&b, "column"));
    let (oa, ob) = (
        a.column_operation.as_ref().expect("operating"),
        b.column_operation.as_ref().expect("operating"),
    );
    for (j, (ta, tb)) in oa
        .stage_temperatures_k
        .iter()
        .zip(&ob.stage_temperatures_k)
        .enumerate()
    {
        assert!(
            (ta - tb).abs() < 1e-6 * tb,
            "{water}: stage {j} at {ta} K against {tb} K"
        );
    }
    assert!((oa.reflux_ratio - ob.reflux_ratio).abs() < 1e-6 * ob.reflux_ratio);
    let (da, db) = (
        a.column_duty.as_ref().expect("duties"),
        b.column_duty.as_ref().expect("duties"),
    );
    assert!(
        (da.reboiler_w - db.reboiler_w).abs() < 1e-6 * db.reboiler_w,
        "{water}: reboiler {} W against {} W",
        da.reboiler_w,
        db.reboiler_w
    );
    assert!((da.condenser_w - db.condenser_w).abs() < 1e-6 * db.condenser_w);
}

#[test]
fn slow_the_column_started_at_2_percent_is_the_column() {
    the_started_column_is_the_column(0.02, 0.7);
}

#[test]
fn slow_the_column_started_at_10_percent_is_the_column() {
    the_started_column_is_the_column(0.10, 0.6);
}

/// **What the retry cannot start is refused with the column's own reason
/// first.** At a reflux of 0.6 on the 2 % crude the trays above the feed run
/// dry whatever the pumparounds (the cold start's "liquid leaving stage 5 …
/// negative"): the run halts on tick 1 with M61's refusal, its settings, and
/// then how the halved starts failed — down to a sixteenth (§70.1: a fallback
/// must not hide a cause, M63.4's review).
#[test]
fn a_column_the_halvings_cannot_start_is_refused_with_its_own_reason() {
    let src = wetter(0.02, 1.0).replace("reflux_ratio = 4.0", "reflux_ratio = 0.6");
    let error = run(&mut build(&src), 1).expect_err("the column cannot run at this reflux");
    let own = error
        .find("could not settle at reflux ratio 0.6 with its pumparounds at 15.0 MW on stage 4")
        .unwrap_or_else(|| panic!("the column's own settings: {error}"));
    let retried = error
        .find("Started again from its pumparounds halved, up to 4 times")
        .unwrap_or_else(|| panic!("the retry: {error}"));
    assert!(
        own < retried,
        "the column's own reason comes first: {error}"
    );
    assert!(
        error.contains("the liquid leaving stage 5"),
        "the physical edge: {error}"
    );
    assert!(
        error.contains("at 0.0625 of them"),
        "the last halving: {error}"
    );
}

fn flow(s: &Snapshot, edge: &str) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == edge)
        .unwrap_or_else(|| panic!("no edge '{edge}'"))
        .stream
        .mass_flow
        .value()
}

/// **A crude half water floods the crude unit, and the run goes on** (M65.2,
/// §70.2). Before M65.2 the flooded column handed its pumped lines the hot feed
/// itself and the naphtha pump could not hold its empty line ("arrives at 0.81
/// bar against the store's 1.05"), then the residue pump could not move the
/// boiling oil. As a flash zone, every tick floods and runs: the water draw takes
/// the crude's water, the oil leaves overhead and below, the side draws stand
/// full and still.
#[test]
fn slow_a_crude_half_water_floods_and_runs() {
    let mut engine = build(&wetter(0.50, 1.0));
    for tick in 1..=300 {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let s = engine.snapshot();
        assert!(
            node(&s, "column").column_flooded.is_some(),
            "tick {tick}: the column floods"
        );
    }
    let s = engine.snapshot();
    let charge = charge(&s);
    let (naphtha, residue, water) = (
        flow(&s, "naphtha_suction"),
        flow(&s, "residue_suction"),
        flow(&s, "water_draw"),
    );
    assert!(
        (naphtha + residue + water - charge).abs() < 1e-6 * charge,
        "{naphtha} + {residue} + {water} kg/s out of {charge}"
    );
    assert!(
        (water - 0.5 * charge).abs() < 1e-6 * charge,
        "the water draw {water}"
    );
    for side in ["kerosene_suction", "diesel_suction", "gas_oil_suction"] {
        assert_eq!(flow(&s, side), 0.0, "{side} stands still");
    }
}

/// **A feed wet enough to flood is not retried** (M65.2, §70.2): at 20 % water
/// the column's cold start refuses and the feed is past `FLOODED_WATER_SHARE`,
/// so it floods on tick 1 — the halvings would have started it (M65.0, finding
/// 6), at half a second a tick on a crude unit that stays flooded.
#[test]
fn a_crude_20_percent_water_floods_without_its_halvings() {
    let mut engine = build(&wetter(0.20, 1.0));
    let s = run(&mut engine, 1).expect("tick 1");
    assert!(
        node(&s, "column").column_flooded.is_some(),
        "the column floods on tick 1"
    );
}

/// **The flooded column's first law closes on the plant's own books** (the wet
/// plant's gate, `crude_unit_wet_reference.rs`, on a flooded tick): the feed's
/// enthalpy as the engine carries it, latent heat included, plus the reboiler,
/// is every draw's enthalpy plus the condenser — no pumparound runs in a column
/// with no trays.
#[test]
fn the_flooded_columns_first_law_closes_on_the_plants_books() {
    let mut engine = build(&wetter(0.50, 1.0));
    let flux = |engine: &Engine, s: &Snapshot, edge: &str| {
        let e = s.edges.iter().find(|e| e.name == edge).unwrap();
        engine
            .enthalpy()
            .stream_enthalpy_flux(&engine.slate, &e.stream)
            .unwrap()
            .value()
    };
    let mut worst = 0.0f64;
    for _ in 0..20 {
        let s = run(&mut engine, 1).expect("it runs");
        let arriving = flux(&engine, &s, "transfer_line__downstream");
        let mut leaving = flux(&engine, &s, "water_draw");
        for product in ["naphtha", "kerosene", "diesel", "gas_oil", "residue"] {
            leaving += flux(&engine, &s, &format!("{product}_suction"));
        }
        let duty = node(&s, "column").column_duty.unwrap();
        let books = arriving + duty.reboiler_w - duty.condenser_w - leaving;
        let gross = arriving.abs() + duty.reboiler_w + duty.condenser_w + leaving.abs();
        worst = worst.max(books.abs() / gross);
    }
    assert!(
        worst <= 1e-12,
        "the flooded column's first law: worst residual {worst:e}"
    );
}

/// The shipped crude with `water` [mass fraction], as a supply's composition.
fn crude_at(water: f64) -> refinery_core::components::Composition {
    let oil: f64 = OIL.iter().map(|(_, f)| f).sum();
    let mut weights: Vec<f64> = OIL.iter().map(|(_, f)| f / oil * (1.0 - water)).collect();
    weights.push(water);
    refinery_core::components::Composition::from_weights(&weights).unwrap()
}

fn set_crude(engine: &mut Engine, water: f64) -> Result<(), String> {
    let node = node(&engine.snapshot(), "crude_source").id;
    engine
        .apply(Command::SetSourceComposition {
            node,
            composition: crude_at(water),
        })
        .map_err(|e| e.to_string())
}

/// **The crude's water by command, into the running plant and out again**
/// (M65.3, §70.3). The shipped wet crude unit (0.3 % water) settled, its crude
/// stepped to 2 % water: the supply boils harder, the charge falls toward the
/// 2 % plant's, the column follows warm, and the water draw carries the crude's
/// water. Stepped back, the charge returns to the shipped plant's.
#[test]
fn slow_the_crudes_water_steps_by_command_and_back() {
    let mut engine = build(WET);
    let shipped = charge(&run(&mut engine, 500).expect("the shipped plant runs"));
    set_crude(&mut engine, 0.02).expect("a wetter crude");
    let wet = run(&mut engine, 600).expect("the wetter crude runs");
    let wet_charge = charge(&wet);
    let water = flow(&wet, "water_draw");
    let mut file = build(&wetter(0.02, 1.0));
    let file_charge = charge(&run(&mut file, 1100).expect("the 2 % plant runs"));
    set_crude(&mut engine, 0.003).expect("the shipped crude");
    let back = charge(&run(&mut engine, 600).expect("the shipped crude again"));
    // Measured: 112.833 kg/s shipped; 75.835 at 2 % (the 2 % plant file's
    // 75.692 at the same tick, its coil's history its own); back to 112.476.
    assert!(node(&wet, "column").column_flooded.is_none());
    assert!(
        (wet_charge / file_charge - 1.0).abs() < 0.005,
        "the charge at 2 %: {wet_charge} kg/s against the 2 % plant's {file_charge}"
    );
    assert!(
        (water - 0.02 * wet_charge).abs() < 1e-6 * wet_charge,
        "the water draw {water} kg/s against the crude's {}",
        0.02 * wet_charge
    );
    assert!(
        (back / shipped - 1.0).abs() < 0.005,
        "back: {back} kg/s against {shipped}"
    );
}

/// **A wet slug past the flood, and the column separating again**: the crude
/// stepped to 50 % water floods the column within ticks (a flash zone, §70.2);
/// stepped back to the shipped crude, it fractionates again.
///
/// **And the one guess on this plant that fails is dropped** (M71.1,
/// docs/DESIGN.md §76.2): on the second tick after the step back, the straight
/// line from the flooded column's draws through the separated ones guesses a
/// state the kerosene line cannot deliver from (it "arrives at −536 748 bar"),
/// so that tick is solved again from its last states, unguessed — measured, the
/// only such tick of the story.
#[test]
fn slow_a_wet_slug_floods_the_column_and_passes() {
    let mut engine = build(WET);
    run(&mut engine, 300).expect("the shipped plant runs");
    set_crude(&mut engine, 0.50).expect("a slug of water");
    let mut flooded_at = None;
    for tick in 1..=200 {
        let s = run(&mut engine, 1).unwrap_or_else(|e| panic!("slug tick {tick}: {e}"));
        if flooded_at.is_none() && node(&s, "column").column_flooded.is_some() {
            flooded_at = Some(tick);
        }
    }
    set_crude(&mut engine, 0.003).expect("the shipped crude");
    let mut separated_at = None;
    let mut dropped = Vec::new();
    for tick in 1..=400 {
        let s = run(&mut engine, 1).unwrap_or_else(|e| panic!("after tick {tick}: {e}"));
        if separated_at.is_none() && node(&s, "column").column_flooded.is_none() {
            separated_at = Some(tick);
        }
        if s.solver.guess_dropped {
            dropped.push(tick);
        }
    }
    // The supply's composition reaches the column on the tick it is set: the
    // lines between are carried quasi-steady.
    assert_eq!(flooded_at, Some(1), "the slug floods the column");
    assert_eq!(separated_at, Some(1), "the column separates again");
    assert_eq!(
        dropped,
        vec![2],
        "the guess dropped where it fails, and only there"
    );
}

/// **A supply's composition is refused by name** where it is not one: the
/// wrong length, a negative fraction, fractions that do not sum to one, a node
/// that is not a supply; where it would change the supply's phase (the vgo
/// feed of the make-up plant turned to hydrogen); and where the liquid would
/// boil at the supply's pressure on a plant that cannot carry a boiling supply
/// (the dry crude unit's crude turned to light naphtha at 270 °C, 30 bar). Each
/// refusal leaves the supply as it was.
#[test]
fn a_supply_composition_is_refused_by_name() {
    use refinery_core::components::Composition;
    let mut engine = build(WET);
    let crude = node(&engine.snapshot(), "crude_source").id;
    let tank = node(&engine.snapshot(), "sour_water_tank").id;
    let raw = |fractions: Vec<f64>| -> Composition {
        serde_json::from_value(serde_json::json!({ "mass_fractions": fractions })).unwrap()
    };
    let refused = |engine: &mut Engine, node, composition| {
        engine
            .apply(Command::SetSourceComposition { node, composition })
            .expect_err("refused")
            .to_string()
    };
    let e = refused(&mut engine, crude, raw(vec![0.5, 0.5]));
    assert!(e.contains("one mass fraction per slate member (7)"), "{e}");
    let e = refused(
        &mut engine,
        crude,
        raw(vec![0.2, 0.2, 0.2, 0.2, 0.2, 0.3, -0.1]),
    );
    assert!(e.contains("each finite and >= 0"), "{e}");
    let e = refused(
        &mut engine,
        crude,
        raw(vec![0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
    );
    assert!(e.contains("summing to 1"), "{e}");
    let e = refused(&mut engine, tank, crude_at(0.02));
    assert!(e.contains("is not a supply"), "{e}");
    assert_eq!(
        serde_json::to_value(&node(&engine.snapshot(), "crude_source").kind).unwrap(),
        serde_json::to_value(&node(&build(WET).snapshot(), "crude_source").kind).unwrap(),
        "the supply stands as it was"
    );

    let mut makeup = build(include_str!("../../../scenarios/hydrocracker_makeup.toml"));
    let vgo = node(&makeup.snapshot(), "vgo_feed").id;
    let hydrogen = Composition::pure(
        makeup.slate.len(),
        makeup.slate.index_of("hydrogen").unwrap(),
    );
    let e = refused(&mut makeup, vgo, hydrogen);
    assert!(e.contains("would change its phase"), "{e}");

    let mut dry = build(include_str!("../../../scenarios/crude_unit.toml"));
    let source = node(&dry.snapshot(), "crude_source").id;
    let light = Composition::pure(dry.slate.len(), 0);
    let e = refused(&mut dry, source, light);
    assert!(e.contains("would be boiling"), "{e}");
}

const WETTER: &str = include_str!("../../../scenarios/crude_unit_wetter.toml");

fn coil_c(s: &Snapshot) -> f64 {
    match &node(s, "furnace").kind {
        refinery_core::graph::NodeKind::Furnace { coil, .. } => coil.temperature.value() - 273.15,
        other => panic!("the furnace has a coil: {other:?}"),
    }
}

/// **The wetter crude unit runs as its file says** (M65.4, §70.4): the wet crude
/// unit on a crude 2 % water, its column started on tick 1 by M65.1's retry at
/// the shipped 40 MW; it stands where it starts — the charge 75.078 kg/s and the
/// coil at its declared 339.79 °C, measured settled over 60 000 ticks — the
/// water draw carrying the crude's 2 %, the column fractionating, not flooded.
#[test]
fn the_wetter_crude_unit_runs_as_its_file_says() {
    let mut engine = build(WETTER);
    let s = run(&mut engine, 300).expect("the wetter crude unit runs");
    let column = node(&s, "column");
    assert!(column.column_operation.is_some() && column.column_flooded.is_none());
    let charge = charge(&s);
    assert!((charge - 75.078).abs() < 0.01, "the charge {charge} kg/s");
    let coil = coil_c(&s);
    assert!((coil - 339.79).abs() < 0.05, "the coil at {coil} °C");
    let water = flow(&s, "water_draw");
    assert!(
        (water - 0.02 * charge).abs() < 1e-6 * charge,
        "the water draw {water} kg/s"
    );
}

/// **Both fidelities agree** on the wetter crude unit.
#[test]
fn slow_the_wetter_crude_unit_runs_on_both_fidelities() {
    let mut answers = Vec::new();
    for solver in ["newton", "simple"] {
        let mut engine =
            build(&WETTER.replace("flow = \"newton\"", &format!("flow = \"{solver}\"")));
        let s = run(&mut engine, 150).unwrap_or_else(|e| panic!("{solver}: {e}"));
        let operation = node(&s, "column")
            .column_operation
            .clone()
            .expect("operating");
        answers.push((
            charge(&s),
            flow(&s, "water_draw"),
            operation.stage_temperatures_k,
        ));
    }
    let (a, b) = (&answers[0], &answers[1]);
    assert!((a.0 - b.0).abs() < 1e-6 * a.0 && (a.1 - b.1).abs() < 1e-6 * a.1);
    for (ta, tb) in a.2.iter().zip(&b.2) {
        assert!((ta - tb).abs() < 1e-4, "newton {ta} K, simple {tb} K");
    }
}
