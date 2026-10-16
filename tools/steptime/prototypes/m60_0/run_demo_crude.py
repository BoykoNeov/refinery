import json, re, subprocess, sys, pathlib
src = pathlib.Path(r"W:/Claude_projects/refinery/scenarios/crude_column_cascade.toml").read_text(encoding="utf-8")
exe = r"W:/temp/claude/m60-probe/target/release/refinery.exe"
out = pathlib.Path(r"W:/temp/claude/m60-scripts/variants"); out.mkdir(exist_ok=True)
solver = sys.argv[1] if len(sys.argv) > 1 else "newton"
for delta in [float(x) for x in sys.argv[2:]]:
    duty = 2.2607 + 0.4009 * delta
    text = re.sub(r"duty_mw = 2\.2607", f"duty_mw = {duty:.4f}", src)
    f = out / f"d{int(delta)}.toml"; f.write_text(text, encoding="utf-8")
    r = subprocess.run([exe, "run", str(f), "--ticks", "3000", "--snapshot-every", "0", "--solver", solver], capture_output=True, text=True)
    probe = [l for l in r.stderr.splitlines() if l.startswith("PROBE")]
    if r.returncode != 0:
        err = [l for l in r.stderr.splitlines() if not l.startswith("PROBE")]
        its = [int(re.search(r"iterations=(\d+)", l).group(1)) for l in probe if "iterations=" in l]
        print(f"delta {delta}: FAILED: {' '.join(err)[-500:]}  max_iter={max(its) if its else None} last probe: {probe[-3:]}"); continue
    snap = json.loads(r.stdout.strip().splitlines()[-1])
    its = [int(re.search(r"iterations=(\d+)", l).group(1)) for l in probe if "iterations=" in l]
    loc = [l for l in probe if "local_reboiler" in l]
    fl = [l for l in probe if "flash:" in l]
    print(f"delta {delta}: max_iter={max(its)} first_iter={its[0]} | {probe[-1]}")
    if loc: print("  ", loc[-1][:160])
    re_ = [l for l in probe if "R_eff" in l]
    if re_: print("  ", re_[-1][:80])
    if fl: print("  ", fl[-1])
    for n in snap["nodes"]:
        if n.get("column_duty"):
            d = n["column_duty"]; print("  duties MW: condenser %.2f reboiler %.2f" % (d["condenser_w"]/1e6, d["reboiler_w"]/1e6))
    for e in snap["edges"]:
        nm = e.get("name", "")
        if nm.startswith(("naphtha_draw", "distillate_draw", "bottoms_draw", "hot_feed_line__downstream")) and not nm.endswith("upstream"):
            st = e["stream"]
            print("   %-28s T=%.1fC flow=%.2f mf=%s" % (nm, st["temperature"] - 273.15, st["mass_flow"], [round(x, 3) for x in st["composition"]["mass_fractions"]]))
