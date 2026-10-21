"""M73.1: the M73.0 probe's sieve-tray hydraulics with the CHECKED set
(docs/DESIGN.md §78.4.1), run on the probe's saved loads. Changed from
`../m73_0/hyd.py`:

- K2: Ogboja & Kuye's fit of Sinnott Fig. 11.30, 23.48 + 1.66 ln(hw + how [mm])
  (Souza et al., Chem. Eng. Res. Des. 180 (2022), eq. 67), not the table
  written from memory;
- the flow parameter's L/G divided by the number of passes (Perry's Chemical
  Engineers' Handbook, 8th ed., p. 14-37), in the rating and in the design;
- the surface tension's corresponding-states exponent 11/9 (Brock & Bird 1955),
  not 1.25.

Everything else as checked and as the probe had it: Fair's chart as fitted by
Lygeros & Magoulas (Perry eq. 14-90), Sinnott's weep point (eq. 11.84), Francis
weir crest, dry plate with C0 0.84 (10 % holes, plate as thick as its holes),
residual head, downcomer backup limited at (TS + hw)/2. The geometry still
takes the weir at 0.77 of the diameter, as the probe did (the engine takes the
downcomer's chord). TB below is hard-coded, as in the probe.
"""
import json
import math
import sys

R = 8.314462618
WATER_MW = 0.018015

def k2(h):
    """Ogboja & Kuye (1990), Sinnott Fig. 11.30; h = hw + how [mm]."""
    return 23.48 + 1.66 * math.log(max(h, 10.0))


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
    with the 11/9 power of the corresponding-states form (Brock & Bird, AIChE J.
    1 (1955) 174); mole-fraction averaged; floored at
    1 mN/m; times `scale` for the sensitivity."""
    sigma = 0.0
    for xi, b in zip(x, tb):
        tc = 1.15 * b + 120.0
        sigma += xi * 27.0 * max(0.0, (tc - t) / (tc - 288.15)) ** (11 / 9)
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


def flood_velocity(tr, ts, passes=1):
    # M73.1 check: Perry 8th ed. 14-37, L/G divided by the number of passes
    flv = (tr["l_mass"] / tr["v_mass"] / passes) * math.sqrt(tr["rho_v"] / tr["rho_l"])
    csb = 0.0105 + 8.127e-4 * (ts * 1000) ** 0.755 * math.exp(-1.463 * flv ** 0.842)
    return flv, csb * (tr["sigma"] / 20.0) ** 0.2 * math.sqrt((tr["rho_l"] - tr["rho_v"]) / tr["rho_v"])


def hydraulics(tr, g):
    flv, unf = flood_velocity(tr, g.ts, g.passes)
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
        for passes in (1, 2, 4):
            _, unf = flood_velocity(tr, 0.61, passes)
            an = tr["qv"] / (flood_target * unf)
            d = math.sqrt(4 * (an / 0.88) / math.pi)
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
