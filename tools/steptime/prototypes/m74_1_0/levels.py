"""M74.1.0 probe: a column whose trays' liquid follows the flow over them, and
whose reflux drum and base levels are held by streams (DESIGN §79.1, scheme A:
Kister 1990 Fig. 16.4e, Table 16.1 p. 497 -- the distillate holds the drum's
level, the reboiler the base's; the side draws and bottoms at their shares, the
reflux at R times the distillate).

TOTAL moles only (compositions are M74.0's): per stage the moles held H_j,

    dH_j/dt = L_in,j + V_{j+1} - V_j + C_in,j + F_j - L_out,j(H_j)

- a tray's liquid out over its weir from what it holds: Sinnott's heads as
  M73 builds them (trays.rs `sieve_tray`), held at the AFTER state's vapour,
  M(L) = rho*[A_a*(h_w + h_ow(L)) + A_dc*(h_w + h_ow(L) + h_t(L) + h_dc(L))],
  h_ow = c1*L^(2/3) (Francis), h_t = dry + h_w + h_ow + resid, h_dc = c2*L^2,
  inverted by bisection; L the liquid leaving the tray (down, drawn and into a
  loop, as M73 loads it). The side draws and loop draws are fixed flows out of
  it; what is left goes down.
- the vapour: no holdup, each stage's the AFTER state's plus the reboiler's
  move from its AFTER value (constant molar overflow on the perturbation).
- the drum: dH_D/dt = V_0 - reflux - D, D = the level loop's output, reflux
  = R*D (R the AFTER state's); the base: dH_B/dt = L_in - B - V_reb, B fixed,
  V_reb = the level loop's output.
- level loops: PI on the level's deviation from normal, as a share of the
  normal holdup, the output a share of the AFTER flow: GAME VALUES, swept.

Starts at the BEFORE state (holdups at its loads, its flows), the key's new
setting at t = 0; explicit Euler at dt = 0.1 s for an hour.

usage: levels.py <driver output> [drum_min_of_D] [base_min_of_B]
"""
import json
import sys

import numpy as np

sys.stdout.reconfigure(encoding="utf-8")
DT = 0.1
T_END = 3600.0
path = sys.argv[1]
DRUM_MIN = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
BASE_MIN = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

runs = {}
for line in open(path, encoding="utf-8"):
    for tag in ("BEFORE", "AFTER"):
        if line.startswith(tag + " "):
            spec, js = line[len(tag) + 1:].split(": ", 1)
            if js.strip():
                runs.setdefault(spec, {})[tag] = json.loads(js)


def stage_arrays(s):
    st = s["stages"]
    g = lambda k: np.array([x[k] for x in st])
    mw = np.array(s["mw"])
    mwl = np.array([float(np.dot(x["x"], mw)) for x in st])
    return dict(V=g("V"), Ldown=g("Ldown"), Ldrawn=g("Ldrawn"), Cin=g("Cin"),
                Cout=g("Cout"), mwl=mwl, reflux=s["reflux"], R=s["R"],
                mw_top=float(np.dot(s["top_x"], mw)))


def feed_of(f):
    """Total molar feed per stage, the steady residual."""
    n = len(f["V"])
    F = np.zeros(n)
    for j in range(n):
        lin = f["reflux"] if j == 0 else f["Ldown"][j - 1]
        vin = f["V"][j + 1] if j + 1 < n else 0.0
        out = f["Ldown"][j] + f["Ldrawn"][j] + f["Cout"][j] + f["V"][j]
        F[j] = out - lin - vin - f["Cin"][j]
    return F


class Tray:
    """One tray's liquid held [kg] against the liquid leaving it [kg/s]."""

    def __init__(self, t):
        L0 = t["liq_kg_s"]
        self.rho, self.aa, self.adc = t["rho_l"], t["active"], t["dca"]
        self.hw, self.dry, self.resid = t["hw"], t["dry"], t["resid"]
        self.c1 = t["crest"] / L0 ** (2.0 / 3.0)
        self.c2 = t["hdc"] / L0 ** 2
        # check: the reconstruction reads the engine's backup at L0
        how = self.c1 * L0 ** (2 / 3)
        total = self.dry + self.hw + how + self.resid
        assert abs(total - t["total"]) < 1e-9, (total, t["total"])
        assert abs(self.hw + how + total + t["hdc"] - t["backup"]) < 1e-9
        # the inverse tabulated (the probe's speed; the engine would bisect)
        self.Lgrid = np.linspace(0.0, 6.0 * L0, 6001)
        self.Mgrid = np.array([self.mass(x) for x in self.Lgrid])

    def mass(self, L):
        how = self.c1 * max(L, 0.0) ** (2.0 / 3.0)
        total = self.dry + self.hw + how + self.resid
        return self.rho * (self.aa * (self.hw + how) + self.adc * (self.hw + how + total + self.c2 * L * L))

    def flow(self, M):
        return float(np.interp(M, self.Mgrid, self.Lgrid))

    def flow_bisect(self, M):
        """The liquid leaving at holdup M: the inverse of mass(), by a fixed
        count of bisections (deterministic, as capped_drop)."""
        if M <= self.mass(0.0):
            return 0.0
        lo, hi = 0.0, 1.0
        while self.mass(hi) < M:
            hi *= 2.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if self.mass(mid) < M:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def lag(self, L):
        """dM/dL at L [s]: the tray's hydraulic time constant."""
        e = 1e-4 * L
        return (self.mass(L + e) - self.mass(L - e)) / (2 * e)


def simulate(b4, af, kc, ti, scheme="A"):
    fa, fb = stage_arrays(af), stage_arrays(b4)
    n = len(fa["V"])
    F = feed_of(fa)
    trays = [Tray(t) for t in af["trays"]]
    # holdups [mol]: trays at the BEFORE loads (its own hydraulics), the base and
    # the drum at their normal levels, sized on the file's (BEFORE) flows.
    H = np.zeros(n)
    for j, t in enumerate(b4["trays"]):
        H[j] = t["rho_l"] * (t["active"] * (t["hw"] + t["crest"]) + t["dca"] * t["backup"]) / fb["mwl"][j]
    B_fix = fa["Ldrawn"][-1]
    base_norm = BASE_MIN * 60.0 * fb["Ldrawn"][-1]
    H[-1] = base_norm
    D_before = fb["V"][0] - fb["reflux"]
    drum_norm = DRUM_MIN * 60.0 * D_before
    HD = drum_norm
    D_after = fa["V"][0] - fa["reflux"]
    Vreb_after = fa["V"][-1]
    # level loops: the output a multiple of the BEFORE (settled) flow, as the
    # design's "the flow the first tick settled on"; the integrals start at 0.
    iD = 0.0
    iB = 0.0
    Vreb_before = fb["V"][-1]
    B_before = fb["Ldrawn"][-1]
    draws = fa["Ldrawn"][:-1] + fa["Cout"][:-1]
    steps = int(T_END / DT)
    rec = []
    for k in range(steps + 1):
        eD = (HD - drum_norm) / drum_norm
        eB = (H[-1] - base_norm) / base_norm
        D = max(0.0, D_before * (1.0 + kc * eD + iD))
        if scheme == "A":
            Vreb = max(0.0, Vreb_before * (1.0 + kc * eB + iB))
            B = B_fix
        else:
            # B: the reboiler held at its duty before the move (its boilup held,
            # an approximation: the duty's vapour moves with the stage's
            # enthalpies); the bottoms holds the base's level.
            Vreb = Vreb_before
            B = max(0.0, B_before * (1.0 + kc * eB + iB))
        reflux = fa["R"] * D
        V = fa["V"] + (Vreb - Vreb_after)
        Lout = np.zeros(n)
        for j in range(n - 1):
            Lout[j] = trays[j].flow(H[j] * fa["mwl"][j]) / fa["mwl"][j]
        down = np.maximum(Lout[:-1] - draws, 0.0)
        cut = np.minimum(Lout[:-1], draws)  # a tray short of its draws gives what it has
        dH = np.zeros(n)
        for j in range(n - 1):
            lin = reflux if j == 0 else down[j - 1]
            dH[j] = lin + V[j + 1] - V[j] + fa["Cin"][j] + F[j] - Lout[j]
        dH[-1] = down[-1] + F[-1] - B - V[-1]
        dHD = V[0] - reflux - D
        if k % 50 == 0:
            mass = float(np.dot(H, fa["mwl"])) + HD * fa["mw_top"]
            rec.append((k * DT, eD, eB, D / D_before, (Vreb / Vreb_before) if scheme == "A" else (B / B_before), mass, cut.sum() / max(draws.sum(), 1e-9)))
        H = H + DT * dH
        HD = HD + DT * dHD
        iD += DT * kc / ti * eD
        iB += DT * kc / ti * eB
    return np.array(rec), trays, fa


def settle(t, y, tol):
    """The last time |y| exceeds tol."""
    bad = np.nonzero(np.abs(y) > tol)[0]
    return t[bad[-1]] if len(bad) else 0.0


print(f"drum {DRUM_MIN:g} min of distillate, base {BASE_MIN:g} min of bottoms (normal levels), dt {DT}")
ref = runs["reflux=4"]["BEFORE"]
fr = stage_arrays(ref)
print("tray  L_kg/s   held_kg  lag_s (dM/dL)")
tr = [Tray(t) for t in ref["trays"]]
for t, T in zip(ref["trays"], tr):
    L = t["liq_kg_s"]
    print(f"{t['stage']:4d} {L:8.1f} {T.mass(L):9.0f} {T.lag(L):7.1f}")
print(f"down the trays, sum of lags {sum(T.lag(t['liq_kg_s']) for t, T in zip(ref['trays'], tr)):.0f} s")
SPECS = [x for x in sys.argv[4].split(",")] if len(sys.argv) > 4 else list(runs)
TUNINGS = [(2.0, 300.0)] if len(sys.argv) > 5 else [(2.0, 300.0), (4.0, 150.0)]
for scheme in ((sys.argv[5],) if len(sys.argv) > 5 else ("A", "B")):
    for kc, ti in TUNINGS:
        print(f"\n== scheme {scheme}: the drum's level by the distillate, the base's by the "
              f"{'reboiler' if scheme == 'A' else 'bottoms'}; gain {kc}, integral {ti:g} s"
              f"{'' if scheme == 'A' else ' (B ends approximately: the flows after the move are A closure)'}")
        print("   levels as a share of the normal holdup: -100 % empty, +100 % full (normal = half)")
        for spec in SPECS:
            d = runs.get(spec)
            if not d or "BEFORE" not in d or spec == "reflux=4":
                continue
            rec, trays, fa = simulate(d["BEFORE"], d["AFTER"], kc, ti, scheme)
            t, eD, eB, D, X, mass, cut = rec.T
            m0 = mass[0]
            flag = ""
            if eD.max() > 1 or eB.max() > 1:
                flag += " FULL"
            if eD.min() < -1 or eB.min() < -1:
                flag += " EMPTY"
            if cut.min() < 0.999:
                flag += " CUT"
            xname = "reb" if scheme == "A" else "bot"
            print(f"{spec:11s} drum {eD.min()*100:+5.0f}..{eD.max()*100:+4.0f}% base {eB.min()*100:+5.0f}..{eB.max()*100:+4.0f}% "
                  f"| D {D.min():.2f}..{D.max():.2f} {xname} {X.min():.2f}..{X.max():.2f} of before "
                  f"| stored {(mass[-1]-m0)/1000:+.1f} t (peak {(mass.max()-m0)/1000:+.1f}/{(mass.min()-m0)/1000:+.1f}){flag}")
