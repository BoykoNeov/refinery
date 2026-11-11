import sys

src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding="utf-8").read()


def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (s.count(old), old[:100])
    s = s.replace(old, new)


rep("//! M74.5.0's probe driver (stages that stop boiling): M74.4's",
    "//! M74.6.0's probe driver (the start from empty): M74.5.0's, with the key\n"
    "//! `tr` (1 on, 0 off: the total-reflux start, a probe switch on the\n"
    "//! ticking thread), each stage's liquid [t] in the trace, and the\n"
    "//! stages holding nothing counted. Was: M74.5.0's probe driver (stages that stop boiling): M74.4's")

# key tr
rep("""                    "valve" => vec![""",
"""                    "tr" => {
                        refinery_solvers::probe::set_total_reflux(v > 0.5);
                        vec![]
                    }
                    "valve" => vec![""")

# reset total reflux at the start of a run
rep("""    let mut session = Session::default();
    assert_eq!(session.load_scenario(src), "null");""",
"""    refinery_solvers::probe::set_total_reflux(false);
    let mut session = Session::default();
    assert_eq!(session.load_scenario(src), "null");""")

# the trace: held per stage, the products
rep("""    let h = &op["holdup"];
    format!(
        "feed {:.3e} V [{}] T [{}] drum {:.3} base {:.3} reb {:.1} R {:.2}",
        fed,
        list("stage_vapour_mol_s", 1.0, 0.0),
        list("stage_temperatures_k", 1.0, 273.15),""",
"""    let h = &op["holdup"];
    let state = &col["kind"]["cascade"]["holdup"]["state"];
    let held = state["stage_liquid"].as_array().map_or(String::from("-"), |l| {
        l.iter().map(|x| format!("{:.1}", x.as_f64().unwrap() / 1e3)).collect::<Vec<_>>().join(",")
    });
    let drum_t = state["drum_liquid"].as_f64().unwrap_or(f64::NAN) / 1e3;
    let products: Vec<String> = ["naphtha_draw", "kerosene_draw", "diesel_draw", "gas_oil_draw", "residue_draw"]
        .iter()
        .map(|name| {
            s["edges"].as_array().unwrap().iter().find(|e| e["name"].as_str() == Some(name))
                .map_or(String::from("?"), |e| format!("{:.1}", e["stream"]["mass_flow"].as_f64().unwrap_or(f64::NAN)))
        })
        .collect();
    format!(
        "feed {:.1} held[t] [{}] drum {:.1} t V [{}] T [{}] drum {:.3} base {:.3} reb {:.1} R {:.2} cond {:.1} MW products {}",
        fed,
        held,
        drum_t,
        list("stage_vapour_mol_s", 1.0, 0.0),
        list("stage_temperatures_k", 1.0, 273.15),""")
rep("""        op["reflux_ratio"].as_f64().unwrap_or(f64::NAN),
    )
}""", """        op["reflux_ratio"].as_f64().unwrap_or(f64::NAN),
        col["column_duty"]["condenser_w"].as_f64().unwrap_or(f64::NAN) / 1e6,
        products.join("/"),
    )
}""")
rep("fn m74_5_0_cold()", "fn m74_6_0_empty()")
open(dst, "w", encoding="utf-8").write(s)
print("ok")
