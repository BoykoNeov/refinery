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
