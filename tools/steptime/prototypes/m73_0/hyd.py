"""M73.0 probe: sieve-tray hydraulics from a converged column's tray loads.

Reads `LOADS <spec>: <json>` lines (the probe hook in heat_balance.rs) and,
for a chosen geometry, gives each tray's % of flood, weep margin, pressure
drop and downcomer backup. Correlations (all SI; the forms and constants written from memory in the
probe and NOT yet checked against the sources named):

- Flooding: Fair's chart (Fair, Petro/Chem Eng. 33(10) 45, 1961) as fitted by
  Lygeros & Magoulas, Hydrocarbon Processing 65(12) 43 (1986):
  C_sb = 0.0105 + 8.127e-4 TS^0.755 exp(-1.463 F_LV^0.842)  [m/s, TS mm]
  u_nf = C_sb (sigma/20)^0.2 sqrt((rhoL - rhoV)/rhoV) on the NET area.
- Weep point: Sinnott (Coulson & Richardson Vol. 6, 4th ed., eq. 11.84):
  u_h,min = (K2 - 0.90 (25.4 - d_h)) / sqrt(rhoV), K2 from Fig. 11.30
  (values below written from memory, NOT yet checked against the figure).
- Weir crest: Francis weir, Sinnott eq. 11.85: h_ow = 750 (Lw/(rhoL lw))^(2/3) mm.
- Dry plate: Sinnott eq. 11.88: h_d = 51 (u_h/C0)^2 rhoV/rhoL mm, C0 0.84.
- Residual head: Sinnott eq. 11.89: h_r = 12.5e3/rhoL mm.
- Downcomer: Sinnott eqs. 11.91-11.92: h_dc = 166 (Lwd/(rhoL Am))^2 mm,
  h_b = (hw + how) + h_t + h_dc, limit 0.5 (TS + hw).
"""
import json
import math
import sys

R = 8.314462618
WATER_MW = 0.018015

# Sinnott Fig. 11.30, K2 against (hw + how) [mm] -- from memory, unverified.
K2_TABLE = [(0, 29.6), (20, 29.9), (40, 30.3), (60, 30.6), (80, 30.8), (100, 30.9), (140, 31.0)]


def k2(h):
    for (a, ka), (b, kb) in zip(K2_TABLE, K2_TABLE[1:]):
        if h <= b:
            return ka + (kb - ka) * (max(h, a) - a) / (b - a)
    return K2_TABLE[-1][1]


class Geometry:
    def __init__(self, diameter, spacing=0.61, weir=0.050, hole=0.005, hole_share=0.10,
                 downcomer_share=0.12, weir_share=0.77, c0=0.84, clearance=0.040, passes=1):
        self.d = diameter
        self.ts = spacing
        self.hw = weir
        self.dh = hole
        self.ac = math.pi * diameter ** 2 / 4
        self.ad = downcomer_share * self.ac
        self.an = self.ac - self.ad
        self.aa = self.ac - 2 * self.ad
        self.ah = hole_share * self.aa
        # A tray of `passes` liquid paths: the liquid split between them, each
        # over its own weir (Kister, Distillation Design (1992) 6.3).
        self.passes = passes
        self.lw = weir_share * diameter
        self.c0 = c0
        self.am = clearance * self.lw


def liquid_density(x, mw, rho, t, expand):
    """Mixture density [kg/m3] by volume additivity; `expand` the slope [kg/m3/K]
    each cut loses from its declared (15 C) density (0 = declared)."""
    mass = [xi * m for xi, m in zip(x, mw)]
    tot = sum(mass)
    vol = 0.0
    for m, r in zip(mass, rho):
        if r <= 0:
            continue
        vol += (m / tot) / max(r - expand * (t - 288.15), 300.0)
    return 1.0 / vol


def surface_tension(x, tb, t, scale):
    """Liquid surface tension [mN/m]: each cut 27 mN/m at 15 C, falling to zero
    at a critical temperature estimated from its boiling point
    (Tc = 1.15 Tb + 120 K: n-hexane 513 against 507, n-decane 634 against 617),
    with the 1.25 power of the corresponding-states form (Brock & Bird, AIChE J.
    1 (1955) 174, its exponent rounded); mole-fraction averaged; floored at
    1 mN/m; times `scale` for the sensitivity."""
    sigma = 0.0
    for xi, b in zip(x, tb):
        tc = 1.15 * b + 120.0
        sigma += xi * 27.0 * max(0.0, (tc - t) / (tc - 288.15)) ** 1.25
    return scale * max(1.0, sigma / sum(x))


def trays(load, tb, expand=0.65, sigma_scale=1.0):
    """Each TRAY's loads (every stage but the reboiler): vapour from below,
    liquid leaving over its weir."""
    p = load["P"]
    mw = load["mw"]
    rho = load["rho"]
    st = load["stages"]
    n = len(st)
    out = []
    for j in range(n - 1):
        below = st[j + 1]
        vy = sum(below["y"])
        ymw = sum(y * m for y, m in zip(below["y"], mw)) / vy
        v_mol = below["V"] + below["Vw"]
        v_mass = below["V"] * ymw + below["Vw"] * WATER_MW
        mbar = v_mass / v_mol
        rho_v = p * mbar / (R * below["T"])
        here = st[j]
        xmw = sum(x * m for x, m in zip(here["x"], mw))
        l_mol = here["Ldown"] + here["Ldrawn"] + here["Cout"]
        l_mass = l_mol * xmw + here["FW"] * WATER_MW
        rho_l = liquid_density(here["x"], mw, rho, here["T"], expand)
        out.append(dict(tray=j + 1, T=here["T"], Tv=below["T"], v_mass=v_mass, l_mass=l_mass,
                        rho_v=rho_v, rho_l=rho_l, sigma=surface_tension(here["x"], tb, here["T"], sigma_scale),
                        qv=v_mass / rho_v))
    return out


def flood_velocity(tr, ts):
    flv = (tr["l_mass"] / tr["v_mass"]) * math.sqrt(tr["rho_v"] / tr["rho_l"])
    csb = 0.0105 + 8.127e-4 * (ts * 1000) ** 0.755 * math.exp(-1.463 * flv ** 0.842)
    return flv, csb * (tr["sigma"] / 20.0) ** 0.2 * math.sqrt((tr["rho_l"] - tr["rho_v"]) / tr["rho_v"])


def hydraulics(tr, g):
    flv, unf = flood_velocity(tr, g.ts)
    un = tr["qv"] / g.an
    uh = tr["qv"] / g.ah
    lpass = tr["l_mass"] / g.passes
    how = 750 * (lpass / (tr["rho_l"] * g.lw)) ** (2 / 3)
    hw = g.hw * 1000
    uh_min = (k2(hw + how) - 0.90 * (25.4 - g.dh * 1000)) / math.sqrt(tr["rho_v"])
    hd = 51 * (uh / g.c0) ** 2 * tr["rho_v"] / tr["rho_l"]
    hr = 12.5e3 / tr["rho_l"]
    ht = hd + hw + how + hr
    dp = 9.81e-3 * ht * tr["rho_l"]
    hdc = 166 * (lpass / (tr["rho_l"] * g.am)) ** 2
    hb = hw + how + ht + hdc
    return dict(flv=flv, flood=un / unf, weep=uh / uh_min, how=how, dp=dp, backup=hb / (500 * (g.ts + g.hw)))


TB = [t + 273.15 for t in (70, 150, 220, 300, 380, 560, 100)]


def design(load, flood_target=0.80, exit_velocity=0.40, backup_target=0.60, **kw):
    """Each tray its own geometry, designed on `load` (the plant file's
    settings): the diameter at `flood_target` of flood, holes 10 % of the active
    area, the downcomer clearance at `exit_velocity` [m/s] under it (Kister,
    Distillation Design (1992) 6.2: under-downcomer velocity 0.45 m/s at most),
    and the fewest liquid passes (1, 2 or 4) that keep its downcomer backup at
    `backup_target` of the limit or under."""
    out = []
    for tr in trays(load, TB[: len(load["mw"])], **kw):
        _, unf = flood_velocity(tr, 0.61)
        an = tr["qv"] / (flood_target * unf)
        d = math.sqrt(4 * (an / 0.88) / math.pi)
        for passes in (1, 2, 4):
            g = Geometry(d, passes=passes)
            g.am = (tr["l_mass"] / passes / tr["rho_l"]) / exit_velocity
            if hydraulics(tr, g)["backup"] <= backup_target:
                break
        out.append(g)
    return out


def one_diameter(per_tray):
    """The whole column at its widest tray's diameter, each tray's downcomer
    clearance kept."""
    d = max(g.d for g in per_tray)
    out = []
    for g in per_tray:
        h = Geometry(d, passes=g.passes)
        h.am = g.am
        out.append(h)
    return out


def evaluate(load, geoms, **kw):
    return [hydraulics(tr, g) | {"T": tr["T"], "v": tr["v_mass"], "l": tr["l_mass"], "qv": tr["qv"]}
            for tr, g in zip(trays(load, TB[: len(load["mw"])], **kw), geoms)]


def read(path):
    loads = {}
    for line in open(path, encoding="utf-8", errors="replace"):
        if line.startswith("LOADS "):
            spec, js = line[6:].split(": ", 1)
            if js.strip():
                loads[spec] = json.loads(js)
    return loads


def results(path):
    out = {}
    for line in open(path, encoding="utf-8", errors="replace"):
        if line.startswith("RESULT "):
            spec, rest = line[7:].split(": ", 1)
            out[spec] = rest.strip()
    return out
