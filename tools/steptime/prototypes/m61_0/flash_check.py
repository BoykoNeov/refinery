"""Hand check: the crude unit's adiabatic feed flash (Trouton K, liquid-only feed
enthalpy, the heat-balance column's own rule) against the share of the feed's
moles the products above the residue need."""
import math

R = 8.314
C = 88.0  # Trouton constant, J/(mol K)
P_ATM = 101325.0
P = 1.8e5
T_REF = 298.15
# name, Tb C, M kg/mol, cp J/(kg K), mass fraction
cuts = [
    ("light_naphtha", 70, 0.095, 2200, 0.08),
    ("heavy_naphtha", 150, 0.13, 2100, 0.14),
    ("kerosene", 220, 0.17, 2050, 0.16),
    ("diesel", 300, 0.22, 2000, 0.20),
    ("gas_oil", 380, 0.30, 1950, 0.14),
    ("residue", 560, 0.55, 1850, 0.28),
]
moles = [w / m for _, _, m, _, w in cuts]
total = sum(moles)
z = [n / total for n in moles]
tb = [t + 273.15 for _, t, _, _, _ in cuts]
mw = [m for _, _, m, _, _ in cuts]
cp = [c for _, _, _, c, _ in cuts]
print(f"products above the residue need {1 - z[-1]:.3f} of the feed's moles "
      f"(residue {z[-1]:.3f} molar, 0.28 by mass)")


def k(i, t, p=P):
    return P_ATM / p * math.exp((C / R) * (1 - tb[i] / t))


def rr(t, p=P):
    ks = [k(i, t, p) for i in range(len(z))]
    f = lambda b: sum(zi * (ki - 1) / (1 + b * (ki - 1)) for zi, ki in zip(z, ks))
    if f(0.0) <= 0:
        return 0.0, z
    if f(1.0) >= 0:
        return 1.0, [zi * ki for zi, ki in zip(z, ks)]
    lo, hi = 0.0, 1.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if f(mid) > 0:
            lo = mid
        else:
            hi = mid
    b = 0.5 * (lo + hi)
    y = [zi * ki / (1 + b * (ki - 1)) for zi, ki in zip(z, ks)]
    s = sum(y)
    return b, [v / s for v in y]


cp_mol = sum(zi * m * c for zi, m, c in zip(z, mw, cp))


def adiabatic(tf, p=P):
    lo, hi = 250.0, tf
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        b, y = rr(mid, p)
        lam = sum(yi * C * tbi for yi, tbi in zip(y, tb))
        paid = cp_mol * (tf - mid) / lam
        if b < paid:
            lo = mid
        else:
            hi = mid
    t = 0.5 * (lo + hi)
    b, y = rr(t, p)
    lam = sum(yi * C * tbi for yi, tbi in zip(y, tb))
    return min(1.0, cp_mol * (tf - t) / lam), t


for tfc in [298, 356, 404, 428, 450, 475, 500, 525, 550, 600]:
    share, t = adiabatic(tfc + 273.15)
    print(f"feed {tfc:4d} C  flashes {share:.3f} of its moles, settling at {t - 273.15:6.1f} C")
