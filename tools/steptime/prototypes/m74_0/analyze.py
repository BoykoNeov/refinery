"""M74.0 probe: how long a column with holdup takes to settle after a key moves.

Reads the driver's BEFORE/AFTER lines (each run's converged column, the tick
before its first move and at its end) and, per component, builds the linear
composition model with the AFTER state's flows and K-values frozen:

    H_j dx_j/dt = (in from above) + (vapour from below) + (loop return) + feed
                  - (down + drawn + loop draw) x_j - V_j K_j x_j

one state per stage plus the reflux drum (a total condenser: the drum's liquid
is the distillate and the reflux). The feed term is inferred per stage as the
steady residual of the AFTER state (checked to sit on the feed stage only).

Holdups [mol]:
- a tray: its liquid on the deck, rho_L * A_active * (h_w + h_ow) (Sinnott's
  clear-liquid head, the one M73's pressure drop uses) plus its downcomer's
  clear-liquid backup, rho_L * A_dc * h_b, at the liquid leaving it;
- the base (the reboiler stage) and the reflux drum: TAU_BASE / TAU_DRUM of
  the liquid leaving them (Ludwig Vol. 1, 4th ed., rules of thumb p. xix,
  "Reflux drums ... liquid holdup of 5 min half-full"; p. xxiv, 5-10 min for a
  product feeding another tower).

Each run: the slowest time constant per component, and the products' step
response from the BEFORE compositions, exact (eigen-decomposition) and by
implicit Euler at the engine's dt, the time to 90 % / 99 % of the move.

usage: analyze.py <driver output> [tau_base_s] [tau_drum_s] [deck_scale]
"""
import json
import sys

import numpy as np

sys.stdout.reconfigure(encoding="utf-8")

DT = 0.1
path = sys.argv[1]
TAU_BASE = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
TAU_DRUM = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0
DECK_SCALE = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0

runs = {}
for line in open(path, encoding="utf-8"):
    for tag in ("BEFORE", "AFTER"):
        if line.startswith(tag + " "):
            spec, js = line[len(tag) + 1:].split(": ", 1)
            js = js.strip()
            if js:
                runs.setdefault(spec, {})[tag] = json.loads(js)


def holdups(s):
    """Moles held per stage (trays, then the base) and in the drum."""
    st = s["stages"]
    n = len(st)
    mw = np.array(s["mw"])
    h = np.zeros(n)
    detail = []
    for tr in s["trays"]:
        j = tr["stage"] - 1
        deck = tr["rho_l"] * tr["active"] * (tr["hw"] + tr["crest"]) * DECK_SCALE
        dc = tr["rho_l"] * tr["dca"] * tr["backup"]
        mwl = float(np.dot(st[j]["x"], mw))
        h[j] = (deck + dc) / mwl
        detail.append((tr["stage"], deck, dc, tr["liq_kg_s"], (deck + dc) / max(tr["liq_kg_s"], 1e-9)))
    base_out = st[-1]["Ldrawn"]
    h[-1] = TAU_BASE * base_out
    drum_out = st[0]["V"]  # total condenser: reflux + distillate = V_top
    h_drum = TAU_DRUM * drum_out
    return h, h_drum, detail


def model(s, h, h_drum):
    """Per component: A (n+1 square, drum first) and b, with dX/dt = A X + b."""
    st = s["stages"]
    n = len(st)
    nc = len(s["mw"])
    K = np.array([x["K"] for x in st])
    X = np.array([x["x"] for x in st])
    V = np.array([x["V"] for x in st])
    Ldown = np.array([x["Ldown"] for x in st])
    Ldrawn = np.array([x["Ldrawn"] for x in st])
    Cin = np.array([x["Cin"] for x in st])
    Cout = np.array([x["Cout"] for x in st])
    reflux = s["reflux"]
    xD = np.array(s["top_x"])
    # Loops: each return stage j takes the liquid of the stage below (j+1).
    loop_src = {j: j + 1 for j in range(n) if Cin[j] > 0}
    for j, k in loop_src.items():
        assert abs(Cin[j] - Cout[k]) < 1e-6 * max(1.0, Cin[j]), (j, Cin[j], Cout[k])
    distillate = V[0] - reflux
    As, bs, feed = [], [], np.zeros((n, nc))
    for c in range(nc):
        m = n + 1
        A = np.zeros((m, m))
        # drum (index 0): in V0 K0 x0, out (reflux + D) xD = V0 xD
        A[0, 1] += V[0] * K[0, c]
        A[0, 0] -= V[0]
        for j in range(n):
            r = j + 1
            if j == 0:
                A[r, 0] += reflux
            else:
                A[r, r - 1] += Ldown[j - 1]
            if j + 1 < n:
                A[r, r + 1] += V[j + 1] * K[j + 1, c]
            if j in loop_src:
                A[r, loop_src[j] + 1] += Cin[j]
            A[r, r] -= Ldown[j] + Ldrawn[j] + Cout[j] + V[j] * K[j, c]
        state = np.concatenate([[xD[c]], X[:, c]])
        resid = A @ state
        b = np.zeros(m)
        b[1:] = -resid[1:]  # the feed, inferred per stage
        feed[:, c] = b[1:]
        drum_resid = resid[0]
        hh = np.concatenate([[h_drum], h])
        As.append(A / hh[:, None])
        bs.append(b / hh)
    return As, bs, feed, distillate, drum_resid


def products(s, Xstate):
    """Mass fractions of each product: drum, side draws, bottoms."""
    st = s["stages"]
    mw = np.array(s["mw"])
    out = {"top": Xstate[:, 0]}
    for j, x in enumerate(st[:-1]):
        if x["Ldrawn"] > 0:
            out[f"s{j}"] = Xstate[:, j + 1]
    out["bot"] = Xstate[:, -1]
    for k, v in out.items():
        w = v * mw
        out[k] = w / w.sum()
    return out


print(f"tau_base {TAU_BASE:g} s, tau_drum {TAU_DRUM:g} s, deck scale {DECK_SCALE:g}, dt {DT}")
first = True
for spec, d in runs.items():
    if "AFTER" not in d:
        print(spec, "no AFTER")
        continue
    a = d["AFTER"]
    h, h_drum, detail = holdups(a)
    if first:
        first = False
        print("tray  deck_kg  downcomer_kg  liquid_kg/s  residence_s")
        for stg, deck, dc, lq, tau in detail:
            print(f"{stg:4d} {deck:8.0f} {dc:12.0f} {lq:11.1f} {tau:11.1f}")
        mw = np.array(a["mw"])
        mwl = [float(np.dot(x["x"], mw)) for x in a["stages"]]
        tray_kg = sum(h[j] * mwl[j] for j in range(len(h) - 1))
        print(f"trays {tray_kg/1000:.1f} t, base {h[-1]*mwl[-1]/1000:.1f} t, "
              f"drum {h_drum*np.dot(a['top_x'], mw)/1000:.1f} t")
    As, bs, feed, dist, drum_resid = model(a, h, h_drum)
    fs = np.abs(feed).sum(axis=1)
    feed_stages = [j for j in range(len(fs)) if fs[j] > 1e-6 * fs.max()]
    taus = []
    for A in As:
        ev = np.linalg.eigvals(A)
        taus.append(-1.0 / ev.real.max())
    nc = len(As)
    worst = max(taus)
    line = f"{spec}\n  feed on stages {feed_stages}, drum residual {drum_resid:.2e}; slowest tau {worst:.0f} s ({worst/60:.1f} min); per component " + " ".join(f"{t:.0f}" for t in taus)
    print(line)
    if "BEFORE" not in d:
        continue
    b4 = d["BEFORE"]
    st0 = np.array([np.concatenate([[b4["top_x"][c]], [x["x"][c] for x in b4["stages"]]]) for c in range(nc)])
    st1 = np.array([np.concatenate([[a["top_x"][c]], [x["x"][c] for x in a["stages"]]]) for c in range(nc)])
    times = np.concatenate([np.arange(0, 600, 5.0), np.arange(600, 14400, 30.0)])
    exact = []
    for c in range(nc):
        A, b = As[c], bs[c]
        xs = -np.linalg.solve(A, b)
        w, Vv = np.linalg.eig(A)
        coef = np.linalg.solve(Vv, st0[c] - xs)
        exact.append(np.real(np.array([xs + Vv @ (coef * np.exp(w * t)) for t in times])))
    exact = np.array(exact)  # (nc, times, states)
    fp_err = np.abs(exact[:, -1, :] - st1).max()
    # implicit Euler at DT to 2 hours, sampled at the same times
    ie = np.zeros_like(exact)
    for c in range(nc):
        A, b = As[c], bs[c]
        M = np.eye(A.shape[0]) - DT * A
        Minv = np.linalg.inv(M)
        x = st0[c].copy()
        t, k = 0.0, 0
        steps = int(round(times[-1] / DT))
        idx = {int(round(tt / DT)): i for i, tt in enumerate(times)}
        for s_ in range(steps + 1):
            if s_ in idx:
                ie[c, idx[s_], :] = x
            x = Minv @ (x + DT * b)
    ie_err = np.abs(ie - exact).max()
    # each product's mass-fraction move, the time to 90 / 99 %
    p0 = products(a, st0)
    p1 = products(a, st1)
    parts = []
    for key in p0:
        tot = np.abs(p1[key] - p0[key]).sum()
        if tot < 1e-6:
            parts.append(f"{key} -")
            continue
        series = [products(a, exact[:, i, :])[key] for i in range(len(times))]
        dev = np.array([np.abs(sv - p1[key]).sum() / tot for sv in series])
        t90 = next((times[i] for i in range(len(times)) if (dev[i:] <= 0.10).all()), float("nan"))
        t99 = next((times[i] for i in range(len(times)) if (dev[i:] <= 0.01).all()), float("nan"))
        parts.append(f"{key} move {tot*100:.2f}pt t90 {t90/60:.1f} t99 {t99/60:.1f} min")
    print(f"  settled vs AFTER {fp_err:.1e}; implicit Euler vs exact {ie_err:.1e}")
    print("  " + " | ".join(parts))
