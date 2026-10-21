"""M73.3: the loop plant's tray sections sized on the SETTLED multipass layout
(docs/DESIGN.md §78.10), on M73.0's saved loads at the file's settings.

The layout (Nutter, in Ludwig's Applied Process Design Vol. 2, 4th ed. (2010),
Fig. 10-149; Kister, Distillation Operation (1990) §6.12 guideline 7; Bolles,
AIChE J. 22 (1976) 153): `downcomer_share` s is the tray's whole downcomer area
over the column's, DCA/At. Every pass has a bubbling area Ab/n and a hole area
Ah/n (so the vapour splits evenly), and its outlet weir tuned to the side
downcomer's chord (so the liquid does too): the rated pass is the side pass —
its weir the chord of a segment s/n of the column, its apron that chord, the
downcomer it feeds from s/n of the column. One pass is the old tray exactly.

The sizing rules (Kister §6.12, §6.16, §6.19): four passes on every section
($M73_PASSES=2 tries two: only the top section, 1–2, meets every rule on two,
and one count is kept down the column — §78.10);
each section's diameter its widest tray's at 80 % of jet flood on At(1 − s);
the narrowest (s, hole share) with every tray's downcomer backup at 80 % of
its limit or under, the liquid's loss under the apron at 1.5 in (38.1 mm) or
under, the weir load at 13 gpm/in or under, the clear liquid in the downcomer
under 0.4 ft/s (Table 6.1, "medium" foaming at 24 in), every side downcomer
4.5 % or more (Nutter), and every tray's holes 1.2 times their weep point or
more. A 50 mm weir, the clearance 45 mm, 5 mm holes.

Usage: size.py [spacing_m]   (default 0.76; reads $M73_OUT/base_loops.out,
default W:/temp/claude/m73_0/out — re-made by M73.0's 8-minute sweep)
Printed at 0.76: the table in §78.10. At 0.61: only the top section meets
every rule (the finding that set 0.76).
"""
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "m73_1"))
import hyd  # noqa: E402  (M73.1's checked set; its loads and correlations)

GPM_PER_IN = 3.785411784e-3 / 60 / 0.0254  # m3/s per m of weir, 1 gpm/in
MAX_WEIR_LOAD = 13.0  # gpm/in
DOWNCOMER_VELOCITY = 0.4 * 0.3048  # m/s
MAX_APRON_LOSS = 38.1  # mm of hot liquid, 1.5 in
MAX_BACKUP = 0.80
MIN_WEEP = 1.2
SIDE_FLOOR = 0.045
FT = 0.3048
SECTIONS = [(1, 2), (3, 5), (6, 8), (9, 10), (11, 11), (12, 12)]
PASSES = int(os.environ.get("M73_PASSES", "4"))
CLEARANCE = 0.045


def chord(share):
    """A segment of `share` of the circle: its chord over the diameter."""
    lo, hi = 0.0, math.pi
    for _ in range(80):
        th = 0.5 * (lo + hi)
        if (th - math.sin(th)) / (2 * math.pi) < share:
            lo = th
        else:
            hi = th
    return math.sin(0.25 * (lo + hi))


class Layout:
    """The settled layout's rated (side) pass."""

    def __init__(self, diameter, share, passes, spacing=0.76, weir=0.050, hole=0.005,
                 hole_share=0.10, clearance=CLEARANCE):
        self.d, self.s, self.passes = diameter, share, passes
        self.ts, self.hw, self.dh = spacing, weir, hole
        self.ac = math.pi * diameter ** 2 / 4
        self.an = self.ac * (1 - share)
        self.aa = self.ac * (1 - 2 * share)
        self.ah = hole_share * self.aa
        self.lw = diameter * chord(share / passes)
        self.c0 = 0.84 + 0.04 * (hole_share - 0.10) / 0.05
        self.ad_side = share * self.ac / passes
        self.am = min(clearance * self.lw, self.ad_side)


def rate(tr, g):
    flv, unf = hyd.flood_velocity(tr, g.ts, g.passes)
    un = tr["qv"] / g.an
    uh = tr["qv"] / g.ah
    lpass = tr["l_mass"] / g.passes
    how = 750 * (lpass / (tr["rho_l"] * g.lw)) ** (2 / 3)
    hw = g.hw * 1000
    uh_min = (hyd.k2(hw + how) - 0.90 * (25.4 - g.dh * 1000)) / math.sqrt(tr["rho_v"])
    hd = 51 * (uh / g.c0) ** 2 * tr["rho_v"] / tr["rho_l"]
    hr = 12.5e3 / tr["rho_l"]
    ht = hd + hw + how + hr
    hdc = 166 * (lpass / (tr["rho_l"] * g.am)) ** 2
    hb = hw + how + ht + hdc
    ql = tr["l_mass"] / tr["rho_l"]
    return dict(flood=un / unf, weep=uh / uh_min, dp=9.81e-3 * ht * tr["rho_l"],
                backup=hb / (500 * (g.ts + g.hw)), apron_loss=hdc,
                weir_load=ql / g.passes / g.lw / GPM_PER_IN,
                dc_velocity=ql / (g.s * g.ac))


def diameter(group, spacing, share):
    d = 0.0
    for tr in group:
        _, unf = hyd.flood_velocity(tr, spacing, PASSES)
        d = max(d, math.sqrt(4 * tr["qv"] / (0.80 * unf) / (1 - share) / math.pi))
    return math.ceil(d * 10) / 10


def design(group, spacing):
    """The narrowest section meeting every rule, or None."""
    best = None
    for share, holes in itertools.product([x / 100 for x in range(18, 41, 2)], (0.10, 0.12, 0.15)):
        if share / PASSES < SIDE_FLOOR:
            continue
        d = diameter(group, spacing, share)
        r = [rate(tr, Layout(d, share, PASSES, spacing=spacing, hole_share=holes)) for tr in group]
        ok = (max(x["backup"] for x in r) <= MAX_BACKUP
              and max(x["apron_loss"] for x in r) <= MAX_APRON_LOSS
              and max(x["weir_load"] for x in r) <= MAX_WEIR_LOAD
              and max(x["dc_velocity"] for x in r) <= DOWNCOMER_VELOCITY
              and min(x["weep"] for x in r) >= MIN_WEEP)
        if ok and (best is None or d < best[0]):
            best = (d, share, holes, r)
    return best


def main():
    spacing = float(sys.argv[1]) if len(sys.argv) > 1 else 0.76
    out = os.environ.get("M73_OUT", "W:/temp/claude/m73_0/out")
    base = next(iter(hyd.read(out + "/base_loops.out").values()))
    trays = hyd.trays(base, hyd.TB[: len(base["mw"])])
    print(f"spacing {spacing} m, {PASSES} passes, clearance {CLEARANCE} m, weir 0.05 m")
    print("section  D m   s     holes | worst: flood  backup  apron mm  gpm/in  weep")
    for a, b in SECTIONS:
        found = design(trays[a - 1:b], spacing)
        if found is None:
            print(f"  {a:>2}-{b:<2}  none meets every rule")
            continue
        d, s, h, r = found
        print(f"  {a:>2}-{b:<2}  {d:4.1f}  {s:.2f}  {h:.2f}  |  {max(x['flood'] for x in r):.2f}"
              f"   {max(x['backup'] for x in r):.2f}    {max(x['apron_loss'] for x in r):5.1f}"
              f"   {max(x['weir_load'] for x in r):5.1f}  {min(x['weep'] for x in r):.2f}")


if __name__ == "__main__":
    main()
