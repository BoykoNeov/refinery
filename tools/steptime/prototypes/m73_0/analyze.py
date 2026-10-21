"""M73.0: where the allowed keys take each tray against a tray designed on the
plant file's settings. Usage: analyze.py <plant> [sigma_scale] [expand]"""
import sys

sys.path.insert(0, "W:/temp/claude/m73_0")
import hyd  # noqa: E402

plant = sys.argv[1]
kw = {}
if len(sys.argv) > 2:
    kw["sigma_scale"] = float(sys.argv[2])
if len(sys.argv) > 3:
    kw["expand"] = float(sys.argv[3])
OUT = "W:/temp/claude/m73_0/out/"
base = next(iter(hyd.read(OUT + f"base_{plant}.out").values()))
sweep = hyd.read(OUT + f"sweep_{plant}.out")
res = hyd.results(OUT + f"sweep_{plant}.out")
fails = [s for s, r in res.items() if not r.startswith("RUNS")]

for label, geoms in (("each tray its own", hyd.design(base, **kw)),
                     ("one diameter", hyd.one_diameter(hyd.design(base, **kw)))):
    b = hyd.evaluate(base, geoms, **kw)
    print(f"=== {plant} ({label}) kw={kw}: {len(sweep)} settings ran, {len(fails)} failed")
    print("  tray  D m  base: T_C  %fl  weep  dP mbar backup | allowed: Qv/base  L/base   max%fl  minweep maxbackup")
    per = []
    for j, (g, h) in enumerate(zip(geoms, b)):
        qv = [hyd.evaluate(ld, geoms, **kw)[j] for ld in sweep.values()]
        rq = [e["qv"] / h["qv"] for e in qv]
        rl = [e["l"] / h["l"] for e in qv]
        per.append(qv)
        print(f"  {j+1:>3} {g.d:5.2f}  {h['T']-273.15:6.1f} {100*h['flood']:5.1f} {h['weep']:5.2f} {h['dp']/100:6.2f} {h['backup']:5.2f} |"
              f" {min(rq):4.2f}-{max(rq):4.2f} {min(rl):4.2f}-{max(rl):4.2f}"
              f"  {100*max(e['flood'] for e in qv):6.1f} {min(e['weep'] for e in qv):6.2f} {max(e['backup'] for e in qv):6.2f}")
    print(f"  base total dP {sum(h['dp'] for h in b)/100:.1f} mbar")
    specs = list(sweep.keys())
    flood = [s for i, s in enumerate(specs) if any(per[j][i]["flood"] > 1.0 for j in range(len(per)))]
    near = [s for i, s in enumerate(specs) if any(per[j][i]["flood"] > 0.9 for j in range(len(per)))]
    weep = [s for i, s in enumerate(specs) if any(per[j][i]["weep"] < 1.0 for j in range(len(per)))]
    dc = [s for i, s in enumerate(specs) if any(per[j][i]["backup"] > 1.0 for j in range(len(per)))]
    dps = [sum(per[j][i]["dp"] for j in range(len(per))) for i in range(len(specs))]
    print(f"  settings: jet flood >100 % on some tray {len(flood)}, >90 % {len(near)}; weeping {len(weep)};"
          f" downcomer backed up {len(dc)}; total dP {min(dps)/100:.0f}-{max(dps)/100:.0f} mbar; of {len(specs)}")
    for name, group in (("flood", flood), ("weep", weep), ("downcomer", dc)):
        if group:
            print(f"   {name} e.g.: {'; '.join(group[:6])}")
if fails:
    print("  failed (ran no answer):", "; ".join(fails[:8]))
