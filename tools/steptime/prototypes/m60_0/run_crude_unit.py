import json, re, subprocess, sys, pathlib, os
exe = r"W:/Claude_projects/refinery/target/release/refinery.exe"
out = pathlib.Path(r"W:/temp/claude/m60-scripts/real"); out.mkdir(exist_ok=True)
cuts = [("light_naphtha", 70, 0.095, 680, 2200), ("heavy_naphtha", 150, 0.130, 750, 2100),
        ("kerosene", 220, 0.170, 800, 2050), ("diesel", 300, 0.220, 850, 2000),
        ("gas_oil", 380, 0.300, 900, 1950), ("residue", 560, 0.550, 980, 1850)]
comp = {"light_naphtha": .08, "heavy_naphtha": .14, "kerosene": .16, "diesel": .20, "gas_oil": .14, "residue": .28}
ticks = int(os.environ.get("TICKS", "2000"))
def plant(feed_c, reflux, pressure, stages, feed_stage, draws):
    s = '[meta]\nname = "probe_crude_unit"\ndescription = "probe"\n[simulation]\ndt = 0.1\n'
    for n, tb, mm, rho, cp in cuts:
        s += f'[[components]]\nname = "{n}"\ntb_c = {tb}.0\nmolar_mass_kg_per_mol = {mm}\ndensity_kg_per_m3 = {rho}.0\ncp_j_per_kg_k = {cp}.0\n'
    s += '[fidelity]\nflow = "newton"\nthermo = "trouton"\nseparation = "heat_balance"\nreactions = "none"\nboiloff = "none"\n'
    cs = ", ".join(f"{k} = {v}" for k, v in comp.items())
    s += f'[nodes.crude_source]\ntype = "source"\npressure_bar = 60.0\ntemperature_c = {feed_c}\ncomposition = {{ {cs} }}\n'
    s += f'[nodes.column]\ntype = "column"\npressure_bar = {pressure}\ndraws = [\n'
    for name, stage, ratio in draws:
        s += f'    {{ outlet = "{name}_tank", stage = {stage}' + (f', draw_ratio = {ratio}' if ratio else '') + ' },\n'
    s += f']\n[nodes.column.cascade]\nstages = {stages}\nfeed_stage = {feed_stage}\nreflux_ratio = {reflux}\npumparounds = [' + os.environ.get('PA', '{ stage = 4, duty_mw = 15.0 }, { stage = 6, duty_mw = 15.0 }, { stage = 7, duty_mw = 10.0 }') + ']\n'
    for name, _, _ in draws:
        s += f'[nodes.{name}_tank]\ntype = "tank"\narea_m2 = 30.0\nheight_m = 30.0\ninitial_level_m = 0.5\ntemperature_c = 40.0\ncomposition = {{ residue = 1.0 }}\n'
    s += '[[pipes]]\nname = "feed_line"\nfrom = "crude_source"\nto = "column"\nlength_m = 50.0\ndiameter_m = 0.07\n'
    for name, _, _ in draws:
        s += f'[[pipes]]\nname = "{name}_draw"\nfrom = "column"\nto = "{name}_tank"\nlength_m = 20.0\ndiameter_m = 0.12\n'
    return s
draws = json.loads(os.environ["DRAWS"]) if "DRAWS" in os.environ else [("naphtha", 0, 0.22), ("kerosene", 3, 0.16), ("diesel", 5, 0.20), ("gas_oil", 7, 0.14), ("residue", 10, None)]
reflux = float(os.environ.get("REFLUX", "2.0")); pressure = float(os.environ.get("PBAR", "1.8"))
for feed_c in [float(x) for x in sys.argv[1:]]:
    f = out / f"t{int(feed_c)}.toml"; f.write_text(plant(feed_c, reflux, pressure, 10, 8, draws), encoding="utf-8")
    r = subprocess.run([exe, "run", str(f), "--ticks", str(ticks), "--snapshot-every", "0"], capture_output=True, text=True)
    probe = [l for l in r.stderr.splitlines() if l.startswith("PROBE")]
    its = [int(re.search(r"iterations=(\d+)", l).group(1)) for l in probe if "iterations=" in l]
    if r.returncode != 0:
        err = [l for l in r.stderr.splitlines() if not l.startswith("PROBE")]
        print(f"feed {feed_c}C: FAILED max_iter={max(its) if its else None}: {' '.join(err)[-300:]}"); [print("   ", l[:300]) for l in probe[-3:]]; continue
    snap = json.loads(r.stdout.strip().splitlines()[-1])
    print(f"feed {feed_c}C:")
    for n in snap["nodes"]:
        if n.get("column_duty"):
            d = n["column_duty"]; print("  duties MW: condenser %.2f reboiler %.2f" % (d["condenser_w"]/1e6, d["reboiler_w"]/1e6), n.get("column_operation"))
    for e in snap["edges"]:
        nm = e.get("name", "")
        if nm.endswith("_draw__downstream") or nm == "feed_line__downstream":
            st = e["stream"]; print("   %-24s T=%.1fC flow=%.2f mf=%s" % (nm, st["temperature"] - 273.15, st["mass_flow"], [round(x, 3) for x in st["composition"]["mass_fractions"]]))
