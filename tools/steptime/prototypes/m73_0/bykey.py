"""M73.0: share of sampled settings flooded / weeping, by each key's value (each tray its own design)."""
import os
import sys, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hyd
plant = sys.argv[1]
OUT = os.environ.get("M73_OUT", "W:/temp/claude/m73_0/out") + "/"
base = next(iter(hyd.read(OUT + f"base_{plant}.out").values()))
geoms = hyd.design(base)
sweep = hyd.read(OUT + f"sweep_{plant}.out")
tab = collections.defaultdict(lambda: [0, 0, 0])
for spec, ld in sweep.items():
    ev = hyd.evaluate(ld, geoms)
    fl = any(e["flood"] > 1.0 for e in ev)
    we = any(e["weep"] < 1.0 for e in ev)
    for kv in spec.split("+"):
        t = tab[kv]
        t[0] += 1; t[1] += fl; t[2] += we
for kv in sorted(tab, key=lambda s: (s.split("=")[0], float(s.split("=")[1]))):
    n, f, w = tab[kv]
    print(f"  {kv:14s} n={n:3d} flooded {f:3d} weeping {w:3d}")
