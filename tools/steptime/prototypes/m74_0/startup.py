"""M74.0 probe: hand estimates for a start from empty, and the heat a tray's
liquid stores as its temperature moves (is energy holdup needed for a running
column?). usage: startup.py <driver output>"""
import json, sys, math
import numpy as np
sys.stdout.reconfigure(encoding="utf-8")
runs = {}
for line in open(sys.argv[1], encoding="utf-8"):
    for tag in ("BEFORE", "AFTER"):
        if line.startswith(tag + " "):
            spec, js = line[len(tag) + 1:].split(": ", 1)
            if js.strip():
                runs.setdefault(spec, {})[tag] = json.loads(js)
a = runs["reflux=4"]["AFTER"]
mw, cp = np.array(a["mw"]), np.array(a["cp"])
st = a["stages"]
def stage_cp(j):  # J/kg/K, mass-averaged
    w = np.array(st[j]["x"]) * mw; w /= w.sum(); return float(w @ cp)
tray_kg = []
for tr in a["trays"]:
    tray_kg.append(tr["rho_l"] * (tr["active"] * (tr["hw"] + tr["crest"]) + tr["dca"] * tr["backup"]))
mwl = lambda j: float(np.dot(st[j]["x"], mw))
base_kg = 300.0 * st[-1]["Ldrawn"] * mwl(len(st) - 1)
drum_kg = 300.0 * st[0]["V"] * float(np.dot(a["top_x"], mw))
liquid = sum(tray_kg) + base_kg + drum_kg
feed = 103.57
print(f"liquid held: trays {sum(tray_kg)/1e3:.1f} t, base {base_kg/1e3:.1f} t, drum {drum_kg/1e3:.1f} t, all {liquid/1e3:.1f} t")
print(f"  = {liquid/feed/60:.1f} min of feed at {feed} kg/s; trays alone {sum(tray_kg)/feed/60:.1f} min")
# heat to warm the liquid from 20 C to its running temperatures
q_liq = sum(tray_kg[j] * stage_cp(j) * (st[j]["T"] - 293.15) for j in range(len(tray_kg)))
q_liq += base_kg * stage_cp(len(st) - 1) * (st[-1]["T"] - 293.15)
q_liq += drum_kg * stage_cp(0) * (313.15 - 293.15)
# the steel: the shell 15 mm (diameters from the file, 0.76 m spacing, 1.2 m
# top and 1.8 m bottom, Ludwig Vol. 1 rules of thumb p. xix), two flat heads,
# decks and supports 60 kg/m2 of column area -- a game estimate.
secs = [(1, 2, 5.9), (3, 5, 7.3), (6, 8, 8.4), (9, 10, 8.5), (11, 11, 7.0), (12, 12, 8.5)]
shell = 0.0; decks = 0.0
for f, t, d in secs:
    n = t - f + 1
    shell += math.pi * d * 0.76 * n; decks += n * math.pi * d * d / 4
shell += math.pi * 5.9 * 1.2 + math.pi * 8.5 * 1.8 + math.pi / 4 * (5.9**2 + 8.5**2)
steel = shell * 0.015 * 7850 + decks * 60.0
t_mean = np.mean([s["T"] for s in st])
q_steel = steel * 500.0 * (t_mean - 293.15)
print(f"steel (estimate) {steel/1e3:.0f} t; heat to running temperature: liquid {q_liq/1e9:.0f} GJ, steel {q_steel/1e9:.0f} GJ")
for mw_ in (20.0, 87.4, 107.4):
    print(f"  at {mw_:.0f} MW: {(q_liq+q_steel)/mw_/1e6/60:.0f} min")
# heat stored as a running column's temperatures move (reflux 4 -> 2, -> 5)
for spec in ("reflux=2", "reflux=5", "flow=0.5", "ret=-30"):
    b, c = runs[spec]["BEFORE"], runs[spec]["AFTER"]
    dq = sum(tray_kg[j] * stage_cp(j) * (c["stages"][j]["T"] - b["stages"][j]["T"]) for j in range(len(tray_kg)))
    dq += base_kg * stage_cp(len(st) - 1) * (c["stages"][-1]["T"] - b["stages"][-1]["T"])
    dts = [c["stages"][j]["T"] - b["stages"][j]["T"] for j in range(len(st))]
    print(f"{spec}: stage temperatures move {min(dts):+.1f}..{max(dts):+.1f} K; heat stored {dq/1e6:+.0f} MJ = {dq/1e6/600:+.2f} MW over 10 min")
