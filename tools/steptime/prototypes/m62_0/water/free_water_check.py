"""M62.0 hand check: a free-water flash.

Oil cuts by Trouton (C = 88 J/(mol K), the engine's), water by Clausius-
Clapeyron with its own latent heat (40.65 kJ/mol at 100 C). Water never
dissolves in the oil: either all of it is vapour (undersaturated), or the vapour
holds water at its saturation pressure and the rest stands as free water.
"""
import math

R = 8.314
C = 88.0
P_ATM = 101325.0
LAMBDA_W = 40650.0
TB_W = 373.15


def psat_oil(tb, t):
    return P_ATM * math.exp((C / R) * (1 - tb / t))


def psat_w(t):
    return P_ATM * math.exp((LAMBDA_W / R) * (1 / TB_W - 1 / t))


print("water vapour pressure, constant 40.65 kJ/mol, against steam tables:")
for tc, table in [(100, 1.0142), (150, 4.762), (200, 15.55), (250, 39.76), (300, 85.88)]:
    print(f"  {tc:3d} C  model {psat_w(tc + 273.15) / 1e5:7.3f} bar  table {table:7.3f} bar"
          f"  ({psat_w(tc + 273.15) / 1e5 / table - 1:+.1%})")


def flash(z_oil, tb, z_w, t, p):
    """Moles per mole of feed: (V, L_oil, W_free, y_w). z_oil sums with z_w to 1."""
    k = [psat_oil(b, t) / p for b in tb]
    kw = psat_w(t) / p
    z_tot = sum(z_oil)
    zn = [zi / z_tot for zi in z_oil]

    def oil_liquid(v):  # L with sum x = 1, x_i = z_i/(L + V K_i); None if all oil is vapour
        if sum(zi / (v * ki) for zi, ki in zip(z_oil, k)) <= 1:
            return None
        lo, hi = 0.0, z_tot
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if sum(zi / (mid + v * ki) for zi, ki in zip(z_oil, k)) > 1 else (lo, mid)
        return 0.5 * (lo + hi)

    # 1. No vapour at all: below the (wet) bubble point.
    wet = z_w > 0
    if sum(ki * x for ki, x in zip(k, zn)) + (kw if wet else 0.0) < 1:
        return 0.0, z_tot, z_w, 0.0
    # 2. Three phases: free water stands, the vapour holds water at its saturation.
    if wet and kw < 1:
        def g(v):
            ell = oil_liquid(v)
            if ell is None:
                return None, None
            return sum(zi * ki / (ell + v * ki) for zi, ki in zip(z_oil, k)) - (1 - kw), ell
        v_max = z_w / kw  # free water runs out here
        lo, hi = 1e-14, min(v_max, 1.0)
        glo, _ = g(lo)
        ghi, ell_hi = g(hi)
        if glo is not None and glo < 0:
            pass  # cannot happen past step 1
        # g falls with V (more vapour strips the light cuts from the liquid):
        # a root below v_max means free water still stands there.
        if ghi is None or ghi < 0:
            for _ in range(200):
                mid = 0.5 * (lo + hi)
                gm, _ = g(mid)
                if gm is not None and gm > 0:
                    lo = mid
                else:
                    hi = mid
            v = 0.5 * (lo + hi)
            _, ell = g(v)
            return v, ell, z_w - v * kw, kw
    # 3. Two phases: every drop of water is vapour.
    def f(beta):
        return sum(zi * (ki - 1) / (1 + beta * (ki - 1)) for zi, ki in zip(z_oil, k)) + z_w / beta
    if f(1.0) >= 0:
        return 1.0, 0.0, 0.0, z_w
    lo, hi = 1e-14, 1.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) > 0 else (lo, mid)
    v = 0.5 * (lo + hi)
    return v, 1 - v, 0.0, z_w / v


# A wet naphtha: heavy naphtha (Tb 150 C) with 2 mol% water, at 1.8 bar.
tb = [150 + 273.15]
for zw in (0.0, 0.02, 0.10):
    print(f"\nheavy naphtha with {zw:.0%} water (molar) at 1.8 bar:")
    for tc in (100, 110, 112, 114, 116, 118, 120, 130, 150, 165, 170, 180, 200):
        v, ell, w, yw = flash([1 - zw], tb, zw, tc + 273.15, 1.8e5)
        print(f"  {tc:3d} C  vapour {v:6.3f}  oil liquid {ell:6.3f}  free water {w:6.4f}  y_water {yw:5.3f}")
print("\ndry heavy naphtha boils at 1.8 bar at", end=" ")
lo, hi = 300.0, 600.0
for _ in range(100):
    mid = 0.5 * (lo + hi)
    lo, hi = (mid, hi) if psat_oil(tb[0], mid) < 1.8e5 else (lo, mid)
print(f"{lo - 273.15:.1f} C; with free water present (P_oil + P_w = P):", end=" ")
lo, hi = 300.0, 600.0
for _ in range(100):
    mid = 0.5 * (lo + hi)
    lo, hi = (mid, hi) if psat_oil(tb[0], mid) + psat_w(mid) < 1.8e5 else (lo, mid)
print(f"{lo - 273.15:.1f} C")


# A wide-boiling wet naphtha: light naphtha, heavy naphtha and kerosene (equal
# moles) with 5 mol% water, at 1.8 bar — the three-phase window is a range.
tb3 = [70 + 273.15, 150 + 273.15, 220 + 273.15]
zw = 0.05
print(f"\nthree-cut naphtha with {zw:.0%} water (molar) at 1.8 bar:")
for tc in [70, 90, 95, 96, 97, 98, 99, 100, 110, 130]:
    v, ell, w, yw = flash([(1 - zw) / 3] * 3, tb3, zw, tc + 273.15, 1.8e5)
    print(f"  {tc:3d} C  vapour {v:6.3f}  oil liquid {ell:6.3f}  free water {w:6.4f}  y_water {yw:5.3f}")
