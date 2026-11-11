import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding="utf-8").read()
def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (s.count(old), old[:80])
    s = s.replace(old, new)
rep("//! M74.6.0's probe driver", "//! M74.6.2 probe (a stage that boils dry), from M74.6.0's probe driver")
rep("""                    "tr" => {
                        refinery_solvers::probe::set_total_reflux(v > 0.5);
                        vec![]
                    }""", """                    "tr" => vec![format!(r#"{{"cmd":"set_total_reflux","node":{column},"on":{}}}"#, v > 0.5)],""")
rep("""    refinery_solvers::probe::set_total_reflux(false);
    let mut session""", "    let mut session")
rep("""    // SPECS_FILE: the specs""", """    if std::env::var("STEEL").is_ok() {
        let before = src.clone();
        src = src.replace("reboiler = \\"held\\" }", "reboiler = \\"held\\", steel = \\"derived\\" }");
        assert!(src != before, "the steel was not put in");
    }
    // SPECS_FILE: the specs""")
rep("fn m74_6_0_empty()", "fn m74_6_2_dry()")
open(dst, "w", encoding="utf-8").write(s)
