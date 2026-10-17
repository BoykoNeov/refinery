//! M62.6a — steam into the column (docs/DESIGN.md §67.18), through the loader
//! and a running plant. The column's steam equations are gated against an
//! independent solve in `refinery-solvers`' `reference/steam_column.rs`; these
//! gates hold the plant's side of it: the steam line is an ordinary pipe whose
//! flow is the steam rate, it never mixes into the feed, its water leaves by
//! the water draw, and the column's first law closes on the plant's own books.
//!
//! The fixture is `scenarios/crude_unit_wet.toml` with a steam supply, a valve
//! and a line into the reboiler (stage 10), built here rather than shipped: the
//! shipped files are the regression anchor.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::Engine;

const WET: &str = include_str!("../../../scenarios/crude_unit_wet.toml");

/// The steam supply at 3.5 bar and 350 °C through a valve at `opening` into
/// stage `stage`.
fn with_steam(opening: f64, stage: u32) -> String {
    let supply = format!(
        r#"[nodes.steam_supply]
type = "source"
pressure_bar = 3.5
temperature_c = 350.0
composition = {{ water = 1.0 }}

[nodes.steam_valve]
type = "valve"
kv = 160.0
opening = {opening}

[nodes.sour_water_tank]
"#
    );
    let pipes = r#"[[pipes]]
name = "steam_header"
from = "steam_supply"
to = "steam_valve"
length_m = 30.0
diameter_m = 0.15

[[pipes]]
name = "stripping_steam"
from = "steam_valve"
to = "column"
length_m = 10.0
diameter_m = 0.15

[[pipes]]
name = "transfer_line"
"#;
    let plant = WET
        .replacen("[nodes.sour_water_tank]\n", &supply, 1)
        .replacen("[[pipes]]\nname = \"transfer_line\"\n", pipes, 1)
        .replacen(
            "pumparounds = [{ stage = 4,",
            &format!("steam = {{ pipe = \"stripping_steam\", stage = {stage} }}\npumparounds = [{{ stage = 4,"),
            1,
        );
    assert_ne!(plant, WET);
    plant
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn refusal(src: &str) -> String {
    let file = match refinery_scenarios::load_str(src) {
        Ok(file) => file,
        Err(e) => return e.to_string(),
    };
    match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("the plant must be refused"),
        Err(e) => e.to_string(),
    }
}

fn run(engine: &mut Engine, ticks: u32) {
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
        assert!(
            engine.snapshot().solver.converged,
            "tick {t} did not converge"
        );
    }
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

const PRODUCTS: [&str; 5] = ["naphtha", "kerosene", "diesel", "gas_oil", "residue"];

/// **The steam's water leaves by the water draw, and the column balances on
/// the feed and the steam together.** The water draw carries every kilogram of
/// the crude's water and of the steam; no product carries any; the draws add up
/// to what came in on both lines. And the steam STRIPS: the column's bottom
/// stands colder than the same plant's without it, every stage under the feed
/// boiling at a lower oil partial pressure.
#[test]
fn the_steam_leaves_by_the_water_draw_and_strips_the_bottom() {
    let mut steamed = build(&with_steam(0.5, 10));
    let mut dry = build(WET);
    run(&mut steamed, 100);
    run(&mut dry, 100);
    let s = steamed.snapshot();
    let water = steamed.slate.free_water().unwrap();
    let steam = edge(&s, "stripping_steam").stream.clone();
    assert!(steam.mass_flow.value() > 0.3, "{:?}", steam.mass_flow);
    assert_eq!(steam.vapour_fraction, Some(1.0), "the steam is all vapour");
    let feed = edge(&s, "transfer_line__downstream");
    let crude_water = feed.stream.mass_flow.value() * feed.stream.composition.fractions()[water];
    let draw = edge(&s, "water_draw").stream.mass_flow.value();
    let want = crude_water + steam.mass_flow.value();
    assert!(
        (draw - want).abs() < 1e-9 * want,
        "the water draw takes {draw} kg/s of the crude's {crude_water} and the steam's {}",
        steam.mass_flow.value()
    );
    let mut out = draw;
    for product in PRODUCTS {
        let suction = edge(&s, &format!("{product}_suction"));
        assert_eq!(
            suction.stream.composition.fractions()[water],
            0.0,
            "{product} carries water"
        );
        out += suction.stream.mass_flow.value();
    }
    let into = feed.stream.mass_flow.value() + steam.mass_flow.value();
    assert!(
        (out - into).abs() < 1e-9 * into,
        "the column sends out {out} kg/s of the {into} it takes"
    );
    let bottom = |s: &Snapshot| {
        *node(s, "column")
            .column_operation
            .as_ref()
            .unwrap()
            .stage_temperatures_k
            .last()
            .unwrap()
    };
    let (with, without) = (bottom(&s), bottom(&dry.snapshot()));
    assert!(
        with < without - 1.0,
        "the steamed bottom stands at {with} K, the dry one at {without} K"
    );
}

/// **The column's first law closes on the plant's own books, the steam's
/// enthalpy among them** (M62.5's gate with a second inlet): the feed's and the
/// steam's enthalpy as the engine carries them — each line's latent heat
/// included — the reboiler in, the condenser and the pumparounds out, every
/// draw's enthalpy out. The counterfactual: without the steam's enthalpy the
/// books miss by far more than the residual. Steam on the reboiler (whose
/// balance is the closure) and on a stripping tray (whose balance is a Newton
/// equation).
#[test]
fn the_steamed_column_closes_its_first_law_on_the_plants_books() {
    for stage in [10, 9] {
        let mut engine = build(&with_steam(0.5, stage));
        let pumparounds: f64 = match &node(&engine.snapshot(), "column").kind {
            NodeKind::Column {
                cascade: Some(spec),
                ..
            } => spec.pumparounds.iter().map(|p| p.duty.value()).sum(),
            _ => panic!("the crude unit's column has a cascade"),
        };
        let flux = |engine: &Engine, e: &EdgeSnapshot| {
            // The stream's own flux, its latent heat included.
            engine
                .enthalpy()
                .stream_enthalpy_flux(&engine.slate, &e.stream)
                .unwrap()
                .value()
        };
        let (mut worst, mut without_steam) = (0.0f64, f64::INFINITY);
        for _ in 0..60 {
            run(&mut engine, 1);
            let s = engine.snapshot();
            let steam = flux(&engine, edge(&s, "stripping_steam"));
            let arriving = flux(&engine, edge(&s, "transfer_line__downstream")) + steam;
            let mut leaving = flux(&engine, edge(&s, "water_draw"));
            for product in PRODUCTS {
                leaving += flux(&engine, edge(&s, &format!("{product}_suction")));
            }
            let duty = node(&s, "column").column_duty.unwrap();
            let books = arriving + duty.reboiler_w - duty.condenser_w - pumparounds - leaving;
            let gross =
                arriving.abs() + duty.reboiler_w + duty.condenser_w + pumparounds + leaving.abs();
            worst = worst.max(books.abs() / gross);
            without_steam = without_steam.min((books - steam).abs() / gross);
        }
        assert!(
            worst <= 1e-12,
            "stage {stage}: the column's first law: worst residual {worst:e}"
        );
        assert!(
            without_steam > 1e-4,
            "stage {stage}: the books close without the steam ({without_steam:e})"
        );
    }
}

/// **A shut steam valve is the plant without steam**: the same crude, the same
/// products, the same duties, to the solver's rounding (the plant has two more
/// nodes in its hydraulic solve).
#[test]
fn a_shut_steam_valve_is_the_plant_without_steam() {
    let mut shut = build(&with_steam(0.0, 10));
    let mut wet = build(WET);
    run(&mut shut, 60);
    run(&mut wet, 60);
    let (a, b) = (shut.snapshot(), wet.snapshot());
    assert_eq!(edge(&a, "stripping_steam").stream.mass_flow.value(), 0.0);
    for name in [
        "crude_line",
        "water_draw",
        "naphtha_suction",
        "residue_suction",
    ] {
        let (x, y) = (edge(&a, name).stream.clone(), edge(&b, name).stream.clone());
        assert!(
            (x.mass_flow.value() - y.mass_flow.value()).abs() <= 1e-9 * y.mass_flow.value(),
            "{name}: {:?} against {:?}",
            x.mass_flow,
            y.mass_flow
        );
        assert!(
            (x.temperature.value() - y.temperature.value()).abs() < 1e-7,
            "{name}"
        );
    }
    let (x, y) = (
        node(&a, "column").column_duty.unwrap(),
        node(&b, "column").column_duty.unwrap(),
    );
    assert!((x.reboiler_w - y.reboiler_w).abs() <= 1e-7 * y.reboiler_w);
}

/// **A steam line running backwards is refused by name**: a supply below the
/// column's pressure would draw from the column a flow no draw accounts for.
#[test]
fn a_steam_line_running_backwards_is_refused() {
    let low = with_steam(0.5, 10).replace("pressure_bar = 3.5", "pressure_bar = 1.2");
    let mut engine = build(&low);
    let mut error = None;
    for _ in 0..20 {
        if let Err(e) = engine.tick() {
            error = Some(e.to_string());
            break;
        }
    }
    let error = error.expect("a backward steam line must be refused");
    assert!(
        error.contains("stripping_steam") && error.contains("runs backwards"),
        "{error}"
    );
}

/// **The loader's refusals, by name** (§67.18): a steam inlet under any model
/// but the heat-balance cascade, on a slate with no free water, on a stage the
/// column does not have, by a pipe that does not exist or does not end at the
/// column.
#[test]
fn the_steam_inlet_is_refused_wherever_it_cannot_work() {
    let plant = with_steam(0.5, 10);
    let e = refusal(&plant.replace("separation = \"heat_balance\"", "separation = \"cascade\""));
    assert!(
        e.contains("steam inlet") && e.contains("heat_balance"),
        "{e}"
    );
    let e = refusal(&with_steam(0.5, 11));
    assert!(e.contains("stage 11") && e.contains("1..=10"), "{e}");
    let e = refusal(&with_steam(0.5, 0));
    assert!(e.contains("stage 0"), "{e}");
    let e = refusal(&plant.replace("pipe = \"stripping_steam\"", "pipe = \"no_such_pipe\""));
    assert!(
        e.contains("no_such_pipe") && e.contains("not a pipe"),
        "{e}"
    );
    let e = refusal(&plant.replace("pipe = \"stripping_steam\"", "pipe = \"steam_header\""));
    assert!(
        e.contains("steam_header") && e.contains("INTO the column"),
        "{e}"
    );
    let e = refusal(&plant.replace("free_water = true\n", "").replace(
        "    { outlet = \"sour_water_tank\", stage = 0, phase = \"water\" },\n",
        "",
    ));
    assert!(e.contains("steam inlet") && e.contains("free_water"), "{e}");
}

/// **Every product cooler's books close with steam on** (M62.4b's gate under a
/// second inlet): each draw line carries `split · (feed + steam)` on every
/// pipe from the first pass of the tick, so the duty each cooler rates is the
/// heat its published stream gives up, `ṁ·(h_in − h_out)`.
#[test]
fn the_product_coolers_books_close_with_steam_on() {
    let mut engine = build(&with_steam(0.5, 10));
    let flux = |engine: &Engine, e: &EdgeSnapshot| {
        engine
            .enthalpy()
            .stream_enthalpy_flux(&engine.slate, &e.stream)
            .unwrap()
            .value()
    };
    let mut worst = 0.0f64;
    for _ in 0..30 {
        run(&mut engine, 1);
        let s = engine.snapshot();
        for product in PRODUCTS {
            let duty = node(&s, &format!("{product}_cooler"))
                .water_cooler
                .expect("a water cooler publishes its duty")
                .duty_w;
            let given = flux(&engine, edge(&s, &format!("{product}_discharge")))
                - flux(&engine, edge(&s, &format!("{product}_rundown")));
            worst = worst.max((duty - given).abs() / duty.abs());
        }
    }
    assert!(worst < 1e-9, "a product cooler's books: worst {worst:e}");
}
