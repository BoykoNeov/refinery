"""M62.5: the engine's wet-trays answer, checked against the reference's own
equations (docs/DESIGN.md §67.17).

On `wet_trays` the engine's vapour floor binds (stage 6's oil vapour at 20 % of
the oil's moles, the reflux raised to hold it), which the walk in
`wet_column.py` does not reach. So the reference solves the same MESH
equations with that floor, from the engine's own converged state
(`engine_start.json`, the engine's `CascadeProfile` and splits) — the start
only has to be near; what is checked is where the independent equations
settle and that their end checks hold.

Run: python -I check_engine.py
"""

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wet_column as wc  # noqa: E402

here = os.path.dirname(os.path.abspath(__file__))
start = json.load(open(os.path.join(here, "engine_start.json")))
case = dict(wc.WET_TRAYS)
cuts = case["cuts"]
nc, n, f = len(cuts), case["stages"], case["feed_stage"] - 1
p = case["pressure"]
oil_mass = case["feed_mass"] * (1.0 - case["water_mass_fraction"])
feed_c = oil_mass * np.array(case["oil_mass_fractions"]) / np.array([c["m"] for c in cuts])
floor = 0.2 * feed_c.sum()
case["floor"] = (6, floor)
wet = {0, 1, 2, 3, 4}

t = np.array(start["temperatures_k"])
x = np.array(start["liquid"])[:, :nc]
v = np.array(start["oil_vapour"])
fw = np.array(start["free_water"])
r = start["reflux_ratio"]
side_stage, side_ratio = case["side_draws"][0]
# liquid totals from the oil mole balance, top-down
d = v[0] / (r + 1.0)
side_rate = side_ratio * oil_mass / float(x[side_stage - 1] @ np.array([c["m"] for c in cuts]))
leaving = np.zeros(n)
down_above = r * d
for j in range(n):
    leaving[j] = down_above + (v[j + 1] if j + 1 < n else 0.0) + (feed_c.sum() if j == f else 0.0) - v[j]
    down_above = leaving[j] - (side_rate if j == side_stage - 1 else 0.0)
water = np.zeros(n)
feed_w = case["feed_mass"] * case["water_mass_fraction"] / wc.WATER["m"]
water[0] = feed_w - fw[n - 1]
for j in range(n - 1):
    water[j + 1] = water[j] + fw[j] - (fw[j - 1] if j else 0.0) - (feed_w if j == f else 0.0)
per = 2 * nc + 3
u = np.zeros(n * per + 1 + 1)
for j in range(n):
    b = j * per
    k = np.array([wc.psat_oil(c, t[j]) / p for c in cuts])
    y = k * x[j] / (k * x[j]).sum()
    u[b:b + nc] = x[j] * leaving[j]
    u[b + nc:b + 2 * nc] = y * v[j]
    u[b + 2 * nc] = t[j]
    u[b + 2 * nc + 1] = max(water[j], 0.0)
    u[b + 2 * nc + 2] = fw[j]
u[n * per] = side_rate
u[-1] = r

out, _ = wc.report(case, wet, u)
checks = dict(
    residual_ok=out["worst_residual"] < 1e-12,
    dry_unsaturated=all(s <= 1 + 1e-9 for j, s in enumerate(out["saturation"]) if j not in wet),
    wet_nonnegative=all(out["free_water_mol_s"][j] >= 0 for j in wet),
    above_floor=min(out["oil_vapour_mol_s"][1:]) >= floor * (1 - 1e-9),
    liquids_positive=min(out["liquid_mol_s"]) > 0,
)
out["checks"] = {k: bool(v) for k, v in checks.items()}
out["engine_temperatures_c"] = [tk - 273.15 for tk in t]
print(json.dumps(out, indent=1))
