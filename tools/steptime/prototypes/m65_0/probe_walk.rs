//! THROWAWAY M65.0 probe (C14): the wet crude unit on a wetter crude. Is the
//! pumparounds' narrow range the column's, or only its cold start's? Started
//! cold inside the range, then the pumparounds walked warm, up and down.
use refinery_core::engine::Engine;
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::Watt;
use refinery_scenarios::{build_engine, load_str};

const WET: &str = include_str!("../../../../scenarios/crude_unit_wet.toml");
const OIL: [(&str, f64); 6] = [
    ("light_naphtha", 0.07976),
    ("heavy_naphtha", 0.13958),
    ("kerosene", 0.15952),
    ("diesel", 0.1994),
    ("gas_oil", 0.13958),
    ("residue", 0.27916),
];
const PA: [f64; 3] = [15.0, 15.0, 10.0];

/// The shipped wet crude unit with `water` [mass fraction] in its crude (the
/// oil scaled to `1 − water`) and its pumparounds at `scale` of the shipped.
pub fn wetter(water: f64, scale: f64) -> String {
    let oil: f64 = OIL.iter().map(|(_, f)| f).sum();
    let parts: Vec<String> = OIL
        .iter()
        .map(|(n, f)| format!("{n} = {:.6}", f / oil * (1.0 - water)))
        .collect();
    let composition = format!("composition = {{ {}, water = {water:.6} }}", parts.join(", "));
    let start = WET.find("composition = { light_naphtha = 0.07976").unwrap();
    let end = start + WET[start..].find('}').unwrap() + 1;
    let mut text = format!("{}{}{}", &WET[..start], composition, &WET[end..]);
    let old = "pumparounds = [{ stage = 4, duty_mw = 15.0 }, { stage = 6, duty_mw = 15.0 }, { stage = 7, duty_mw = 10.0 }]";
    assert!(text.contains(old));
    text = text.replace(
        old,
        &format!(
            "pumparounds = [{{ stage = 4, duty_mw = {} }}, {{ stage = 6, duty_mw = {} }}, {{ stage = 7, duty_mw = {} }}]",
            PA[0] * scale, PA[1] * scale, PA[2] * scale
        ),
    );
    text
}

pub fn engine(water: f64, scale: f64) -> Engine {
    build_engine(&load_str(&wetter(water, scale)).expect("parses")).expect("builds")
}

fn run(engine: &mut Engine, ticks: u32) -> Result<Snapshot, String> {
    for tick in 0..ticks {
        engine.tick().map_err(|e| format!("tick {tick}: {e}"))?;
    }
    Ok(engine.snapshot())
}

fn column(engine: &Engine) -> refinery_core::graph::NodeId {
    engine.snapshot().nodes.iter().find(|n| n.name == "column").unwrap().id
}

fn set_scale(engine: &mut Engine, scale: f64) {
    let node = column(engine);
    for (i, mw) in PA.iter().enumerate() {
        engine
            .apply(Command::SetPumparoundDuty { node, pumparound: i as u32, duty: Watt((mw * scale * 1e6).max(0.0)) })
            .unwrap();
    }
}

/// Walk from `start` by `step` until a step fails or `stop` is passed; the last
/// scale that ran and the failure's text.
fn walk(water: f64, start: f64, step: f64, stop: f64) -> (f64, String) {
    let mut engine = engine(water, start);
    run(&mut engine, 1000).expect("starts inside the range");
    let mut scale = start;
    loop {
        let next = scale + step;
        if (step > 0.0 && next > stop + 1e-9) || (step < 0.0 && next < stop - 1e-9) {
            return (scale, "reached the end".into());
        }
        set_scale(&mut engine, next);
        if let Err(e) = run(&mut engine, 300) {
            return (scale, e.chars().take(260).collect());
        }
        scale = next;
    }
}

#[test]
fn m65_probe_walk() {
    for (water, start) in [(0.02, 0.7), (0.03, 0.6), (0.05, 0.6), (0.10, 0.6)] {
        let (up, why_up) = walk(water, start, 0.1, 3.0);
        let (down, why_down) = walk(water, start, -0.05, 0.0);
        println!("water {water}: start {start}; warm up to {up:.2} ({why_up})");
        println!("water {water}: start {start}; warm down to {down:.2} ({why_down})");
    }
}
