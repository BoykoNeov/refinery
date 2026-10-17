"""M62.2 hand check: the four-state free-water flash, in MASS terms.

The reference the engine's three-state flash (crates/solvers/src/line_flash.rs,
`free_water_flash`) is gated against in crates/solvers/tests/reference/
free_water.rs. Written by a DIFFERENT route from the engine's: the engine solves
the oil's Rachford-Rice at its partial pressure P - P_w; this script bisects on
the vapour's moles V with the oil liquid found by a nested bisection (as
M62.0's free_water_check.py does), and tests the "oil all vapour over free
water" state explicitly, where M62.0's script reached it only by accident.

Oil cuts by Trouton (C = 88 J/(mol K), the engine's), water by Clausius-
Clapeyron on its own declared latent heat (40.65 kJ/mol at 100 C). Water never
dissolves in the oil, nor the oil in water.

Prints, per case: the state, the vapour's share of the MOLES and of the MASS,
and the free water's share of the mass.
"""
import math

R = 8.314_462_618_153_24  # the engine's R_GAS (CODATA 2018)
C = 88.0
P_ATM = 101325.0
LAMBDA_W = 40650.0
TB_W = 373.15
M_W = 0.018


def psat_oil(tb, t):
    return P_ATM * math.exp((C / R) * (1 - tb / t))


# Water's saturation pressure since M63.2 (docs/DESIGN.md §68.2): IAPWS-IF97's
# saturation line (IAPWS R7-97(2012), eq. 30) between 273.15 and 647.096 K,
# ln P continued linear in 1/T outside it on the curve's own end slopes -- the
# engine's `thermo::water_saturation_pressure`. (Before M63.2: Clausius-
# Clapeyron on the declared 40 650 J/mol.)
IF97_N = (0.11670521452767e4, -0.72421316703206e6, -0.17073846940092e2,
          0.12020824702470e5, -0.32325550322333e7, 0.14915108613530e2,
          -0.48232657361591e4, 0.40511340542057e6, -0.23855557567849,
          0.65017534844798e3)
IF97_RANGE = (273.15, 647.096)
IF97_END_SLOPES = (-5422.112747837726, -5088.781405301052)


def _if97_psat(t):
    n = IF97_N
    th = t + n[8] / (t - n[9])
    a = th * th + n[0] * th + n[1]
    b = n[2] * th * th + n[3] * th + n[4]
    c = n[5] * th * th + n[6] * th + n[7]
    return (2.0 * c / (-b + math.sqrt(b * b - 4.0 * a * c))) ** 4 * 1.0e6


def psat_w(t):
    low, high = IF97_RANGE
    if t < low:
        end, slope = low, IF97_END_SLOPES[0]
    elif t > high:
        end, slope = high, IF97_END_SLOPES[1]
    else:
        return _if97_psat(t)
    return _if97_psat(end) * math.exp(slope * (1.0 / t - 1.0 / end))



def bisect(f, lo, hi, n=200):
    """Root of a function positive at lo and negative at hi."""
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        if f(mid) > 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def flash(z_oil, tb, z_w, t, p):
    """(state, V, oil liquid moles per cut, free water moles, vapour moles per cut,
    water vapour moles), per mole of feed. sum(z_oil) + z_w = 1."""
    k = [psat_oil(b, t) / p for b in tb]
    kw = psat_w(t) / p
    s = sum(z_oil)
    if s == 0:
        return ("pure water", 1.0 if kw > 1 else 0.0, [], z_w if kw <= 1 else 0.0, [], z_w if kw > 1 else 0.0)
    zn = [zi / s for zi in z_oil]
    # 1. No vapour: below the wet bubble point.
    if sum(ki * x for ki, x in zip(k, zn)) + kw <= 1:
        return ("no vapour", 0.0, list(z_oil), z_w, [0.0] * len(z_oil), 0.0)
    if kw < 1:
        # 2b. Oil all vapour over free water: the vapour V = s/(1 - kw) holds
        # every oil mole and water at its saturation; the oil's dew condition at
        # its partial pressure, sum(y_i / K_i) <= 1 with y_i = z_i / V.
        v = s / (1 - kw)
        if v * kw < z_w and sum((zi / v) / ki for zi, ki in zip(z_oil, k)) <= 1:
            return ("oil all vapour", v, [0.0] * len(z_oil), z_w - v * kw, list(z_oil), v * kw)

        # 2a. Three phases: the oil liquid L(V) with sum x = 1, x_i = z_i/(L + V K_i).
        def oil_liquid(v):
            return bisect(lambda ell: sum(zi / (ell + v * ki) for zi, ki in zip(z_oil, k)) - 1, 0.0, s)

        def g(v):  # sum of the oil's vapour fractions minus its share 1 - kw
            ell = oil_liquid(v)
            return sum(zi * ki / (ell + v * ki) for zi, ki in zip(z_oil, k)) - (1 - kw)

        v_max = min(z_w / kw, s / (1 - kw))  # water runs out, or the oil liquid does
        if g(v_max) < 0:
            v = bisect(g, 1e-15, v_max)
            ell = oil_liquid(v)
            liq = [zi * ell / (ell + v * ki) for zi, ki in zip(z_oil, k)]
            vap = [zi - li for zi, li in zip(z_oil, liq)]
            return ("three phases", v, liq, z_w - v * kw, vap, v * kw)

    # 3. Water all vapour: sum_oil z(K-1)/(1 + b(K-1)) + z_w/b = 0.
    def f(b):
        return sum(zi * (ki - 1) / (1 + b * (ki - 1)) for zi, ki in zip(z_oil, k)) + z_w / b

    if f(1.0) >= 0:
        return ("all vapour", 1.0, [0.0] * len(z_oil), 0.0, list(z_oil), z_w)
    b = bisect(f, max(z_w, 1e-15), 1.0)
    liq = [zi / (1 + b * (ki - 1)) * (1 - b) for zi, ki in zip(z_oil, k)]
    vap = [zi - li for zi, li in zip(z_oil, liq)]
    return ("water all vapour", b, liq, 0.0, vap, z_w)


def report(label, z_oil, tb, m_oil, z_w, cases):
    print(f"\n{label}")
    feed_mass = sum(zi * mi for zi, mi in zip(z_oil, m_oil)) + z_w * M_W
    for tc, p in cases:
        state, v, liq, w, vap, wv = flash(z_oil, tb, z_w, tc + 273.15, p)
        vapour_mass = sum(vi * mi for vi, mi in zip(vap, m_oil)) + wv * M_W
        print(f"  {tc:7.2f} C {p / 1e5:4.2f} bar  {state:17s}  V {v:.10f}  "
              f"vapour mass {vapour_mass / feed_mass:.10f}  free water mass {w * M_W / feed_mass:.10f}")


def wet_bubble(z_oil, tb, z_w, p):
    s = sum(z_oil)
    return bisect(lambda t: p - (sum(zi / s * psat_oil(b, t) for zi, b in zip(z_oil, tb)) + psat_w(t)), 250.0, 700.0) - 273.15


HN = [150 + 273.15]
HN_M = [0.130]
P = 1.8e5
report("heavy naphtha, 2 mol% water, 1.8 bar", [0.98], HN, HN_M, 0.02,
       [(t, P) for t in (100.0, 112.0, 120.0, 150.0, 170.0)])
report("heavy naphtha, 90 mol% water, 1.8 bar (oil all vapour over free water)", [0.10], HN, HN_M, 0.90,
       [(t, P) for t in (105.0, 112.0, 115.0, 118.0, 125.0)])
THREE = [70 + 273.15, 150 + 273.15, 220 + 273.15]
THREE_M = [0.095, 0.130, 0.165]
report("three-cut naphtha, 5 mol% water, 1.8 bar", [0.95 / 3] * 3, THREE, THREE_M, 0.05,
       [(t, P) for t in (90.0, 97.0, 98.5, 99.0, 99.3, 99.5, 110.0, 130.0, 200.0)])
print("\nwet bubble points:")
print(f"  heavy naphtha over water, 1.8 bar: {wet_bubble([0.98], HN, 0.02, P):.6f} C")
print(f"  three-cut naphtha over water, 1.8 bar: {wet_bubble([0.95 / 3] * 3, THREE, 0.05, P):.6f} C")
print(f"  pure water, 1.8 bar: {bisect(lambda t: P - psat_w(t), 250.0, 700.0) - 273.15:.6f} C")

# The wet rundown's junction (scenarios/wet_rundown.toml), at the state the
# engine settles it on after 2 000 ticks on the Newton solver — the reference
# crates/scenarios/tests/wet_rundown_reference.rs holds the plant to. The feed is
# the supply's 67.9 / 29.1 / 3.0 mass % of light naphtha, heavy naphtha, water.
# M62.3 moved the state (the tank's outlet draws its water first, so the tank
# holds less of it and its level and back-pressure shift): 358.7392954 K and
# 150 476.65 Pa at M62.2, the line below since.
mass = [0.679, 0.291, 0.03]
molar_mass = [0.100, 0.130, M_W]
moles = [w / m for w, m in zip(mass, molar_mass)]
z = [n / sum(moles) for n in moles]
report("wet rundown junction, at the engine's (T, P)", z[:2], [80 + 273.15, 150 + 273.15], [0.100, 0.130], z[2],
       [(358.8554590812895 - 273.15, 150410.34604117888)])
