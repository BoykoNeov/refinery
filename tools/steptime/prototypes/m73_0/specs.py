"""M73.0: a coarse sample of each plant's ALLOWED key settings (the column
screen's measured tables), one jump each. Usage: specs.py <plant> > specs.txt"""
import itertools
import json
import sys

PLANTS = {
    "loops": ("column_region_loops.json", {"furnace": [0, 10, 20, 30, 40], "reflux": [0.5, 1, 2, 4, 8, 15],
                                           "flow": [0, 0.5, 1, 1.5, 2], "ret": [-60, 0, 60]}),
    "crude": ("column_region_crude.json", {"furnace": [0, 10, 20, 30, 40], "reflux": [0.5, 1, 2, 4, 8, 15],
                                           "pa": [0, 0.5, 1, 2, 3, 4]}),
    "stripped": ("column_region_stripped.json", {"furnace": [37.5, 45, 50, 55, 60, 65], "steam": [0.05, 0.25, 0.5, 0.75, 1.0],
                                                 "pa_mw": [0, 3, 6, 9, 12]}),
}
DRIVER_KEY = {"pa_mw": "pa"}

plant = sys.argv[1]
fname, pick = PLANTS[plant]
region = json.load(open("W:/Claude_projects/refinery/demo/" + fname))
keys = region["keys"]
grid = region["grid"]
specs = []
for combo in itertools.product(*[pick[k] for k in keys]):
    idx = [grid[k].index(float(v)) for k, v in zip(keys, combo)]
    cell = region["allowed"]
    for i in idx[:-1]:
        cell = cell[i]
    if cell[idx[-1]] != "1":
        continue
    specs.append("+".join(f"{DRIVER_KEY.get(k, k)}={float(v)!r}" for k, v in zip(keys, combo)))
print(";".join(specs))
print(len(specs), file=sys.stderr)
