"""M62.4b: a counter-current water cooler condensing a boiling naphtha, rated
by zone analysis (Kern, Process Heat Transfer, 1950, ch. 12) — outside the
engine, on its own flash, to hold `energy::zone_duty` to (docs/DESIGN.md
§67.13).

The stream: the product_cooler plant's two-cut naphtha (Trouton K-values,
C = 88 J/(mol K), the engine's datum h = sum(w cp (T - 273.15)) plus the
vapour's latent heat per kilogram of vapour times the vapour's mass share).
Its state at the cooler's inlet is the isenthalpic flash of what arrived, at
the cooler's pressure. The water: liquid, cp 4186 J/(kg K), counter-current.

Run:  python zone_cooler.py
"""

import math

R = 8.314_462_618_153_24  # the engine's R_GAS
C = 88.0
P_ATM = 101325.0
T_REF = 273.15
CP_W = 4186.0

TB = [80.0 + 273.15, 150.0 + 273.15]
M = [0.100, 0.130]
CP = [2200.0, 2100.0]
W = [0.7, 0.3]  # mass fractions

# The engine's state at tick 2000 of scenarios/product_cooler.toml (printed
# from its snapshot): what arrives at the cooler, and the cooler's pressure.
MASS = 8.40623823853848
T_ARRIVING = 379.7641134946459
LATENT_ARRIVING = 72954.68466717555
P = 141593.34670708922
UA = 40_000.0
WATER = 20.0
T_WATER = 303.15

z = [w / m for w, m in zip(W, M)]
z = [x / sum(z) for x in z]


def k_value(tb, t):
    # Clausius-Clapeyron on Trouton's latent heat, P_sat(tb) = P_atm.
    return P_ATM / P * math.exp(C * tb / R * (1.0 / tb - 1.0 / t))


def sensible(t):
    return sum(w * cp for w, cp in zip(W, CP)) * (t - T_REF)


def flash(t):
    """Vapour mass share and latent heat per kg of vapour at (t, P)."""
    k = [k_value(tb, t) for tb in TB]
    if sum(zi * ki for zi, ki in zip(z, k)) <= 1.0:
        return 0.0, 0.0
    if sum(zi / ki for zi, ki in zip(z, k)) <= 1.0:
        beta = 1.0
    else:
        lo, hi = 0.0, 1.0
        for _ in range(55):
            mid = 0.5 * (lo + hi)
            f = sum(zi * (ki - 1) / (1 + mid * (ki - 1)) for zi, ki in zip(z, k))
            if f > 0:
                lo = mid
            else:
                hi = mid
        beta = 0.5 * (lo + hi)
    x = [zi / (1 + beta * (ki - 1)) for zi, ki in zip(z, k)]
    y = [ki * xi for ki, xi in zip(k, x)]
    mv = sum(yi * mi for yi, mi in zip(y, M))
    ml = sum(xi * mi for xi, mi in zip(x, M))
    q = beta * mv / (beta * mv + (1 - beta) * ml)
    latent = sum(yi * C * tb for yi, tb in zip(y, TB)) / mv
    return q, latent


def temperature_at(h):
    """The isenthalpic flash's temperature for specific enthalpy h [J/kg]."""
    cp = sum(w * c for w, c in zip(W, CP))
    t_le = T_REF + h / cp
    lo, hi = 200.0, t_le
    for _ in range(55):
        mid = 0.5 * (lo + hi)
        q, lam = flash(mid)
        if sensible(mid) + q * lam - h > 0:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


H_IN = sensible(T_ARRIVING) + LATENT_ARRIVING
T_HOT_IN = temperature_at(H_IN)


def area_needed(duty, zones):
    step = duty / zones
    cw = WATER * CP_W
    hot, cold = T_HOT_IN, T_WATER + duty / cw
    area = 0.0
    for j in range(1, zones + 1):
        nh = temperature_at(H_IN - step * j / MASS)
        nc = T_WATER + duty * (zones - j) / (zones * cw)
        a, b = hot - cold, nh - nc
        if a <= 0 or b <= 0:
            return math.inf
        lmtd = 0.5 * (a + b) if abs(a - b) <= 1e-12 * max(a, b) else (a - b) / math.log(a / b)
        area += step / lmtd
        hot, cold = nh, nc
    return area


def duty(zones):
    ceiling = min(MASS * (H_IN - sensible(T_WATER)), WATER * CP_W * (T_HOT_IN - T_WATER))
    lo, hi = 0.0, ceiling
    for _ in range(45):
        mid = 0.5 * (lo + hi)
        if area_needed(mid, zones) < UA:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


if __name__ == "__main__":
    print(f"inlet: {T_HOT_IN - 273.15:.4f} C, vapour {flash(T_HOT_IN)[0]:.6f}")
    reference = duty(160)
    for zones in (2, 5, 10, 20, 40, 80, 160):
        q = duty(zones)
        print(f"K = {zones:4d}: duty {q:.4f} W  ({(q / reference - 1) * 100:+.4f} % of K = 160)")
