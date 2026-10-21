"""M73.0: one key moved alone from the plant file's settings — the worst tray's
% of flood, weep margin and downcomer backup (each tray its own design).
Usage: lines.py <plant>   (reads $M73_OUT/base_<plant>.out and line_<plant>.out)"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hyd  # noqa: E402  (this folder's: the checked set)

plant = sys.argv[1]
OUT = os.environ.get("M73_OUT", "W:/temp/claude/m73_0/out") + "/"
base = next(iter(hyd.read(OUT + f"base_{plant}.out").values()))
geoms = hyd.design(base)
for spec, ld in hyd.read(OUT + f"line_{plant}.out").items():
    ev = hyd.evaluate(ld, geoms)
    fl = max(range(len(ev)), key=lambda j: ev[j]["flood"])
    we = min(range(len(ev)), key=lambda j: ev[j]["weep"])
    bk = max(range(len(ev)), key=lambda j: ev[j]["backup"])
    dp = sum(e["dp"] for e in ev)
    print(f"  {spec:42s} flood {100*ev[fl]['flood']:5.1f} % (tray {fl+1:2d})  weep {ev[we]['weep']:4.2f} (tray {we+1:2d})"
          f"  backup {ev[bk]['backup']:4.2f} (tray {bk+1:2d})  dP {dp/100:5.0f} mbar")
