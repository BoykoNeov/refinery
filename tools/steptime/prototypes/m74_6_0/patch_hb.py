import sys

p = sys.argv[1]
s = open(p, encoding="utf-8").read()


def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (s.count(old), old[:100])
    s = s.replace(old, new)


rep("""    Ok(HoldupState {
        stage_liquid,
        stage_composition,""", """    let mut state = HoldupState {
        stage_liquid,
        stage_composition,""")
rep("""        last_bottoms: settled.bottoms,
        last_reboiler: settled.reboiler,
    })
}""", """        last_bottoms: settled.bottoms,
        last_reboiler: settled.reboiler,
    };
    // M74.6.0 probe: the start from empty. The settled solve above kept for
    // the level loops' references and as each stage's label; the column then
    // holds nothing, its steel and the drum at ambient.
    if crate::probe::empty() {
        let ambient = Kelvin(crate::probe::ambient_k());
        for held in &mut state.stage_liquid {
            *held = Kg(0.0);
        }
        state.drum_liquid = Kg(0.0);
        for t in &mut state.stage_temperature {
            *t = ambient;
        }
        state.drum_temperature = ambient;
        for v in &mut state.tray_vapour {
            *v = KgPerSec(0.0);
        }
    }
    Ok(state)
}""")
open(p, "w", encoding="utf-8").write(s)
print("ok")
