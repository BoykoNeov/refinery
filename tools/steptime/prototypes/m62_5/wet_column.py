"""M62.5 reference: a column whose feed carries free water (docs/DESIGN.md §67.17).

An INDEPENDENT solve of the heat-balance cascade with water, written from the
MESH equations rather than from the engine's arrangement of them:

  unknowns per stage j   l_jc, v_jc   oil component flows leaving as liquid / vapour
                         T_j          the stage temperature
                         W_j          water leaving as vapour
                         FW_j         free water leaving downward (zero on a dry stage)
  plus the molar rate of every side draw.

  M   l_{j-1,c}·(down share) + v_{j+1,c} + f_jc = l_jc + v_jc            each cut
  E   v_jc = K_c(T_j, P)·x_jc·(Σ_c v_jc + W_j)                           Raoult at the TOTAL
                                                                         pressure over the
                                                                         whole vapour, water in it
  water  W_{j+1} + FW_{j-1} + Fw_j = W_j + FW_j
  dry    FW_j = 0                (and checked: W_j/V_j <= Pw(T_j)/P)
  wet    W_j·P = Pw(T_j)·V_j     (and checked: FW_j >= 0)
  H   every stage but the reboiler, water's enthalpy in every stream
  draws  D·M(y_1) = r_0·F_oil,   U_k·M(x_k) = r_k·F_oil   (shares of the OIL fed)

The engine instead takes each cut's K at the oil's partial pressure, sweeps the
liquids by Thomas, and derives the water vapour from the free water top-down;
the two meet only if both are right. Total condenser: oil and water condense at
the WET bubble point of the overhead (Σ K_c·y_c + Pw/P = 1), the water decants
to its own draw, the reflux R·D and the distillate D are oil.

Properties are the engine's (Trouton, `crates/solvers/src/thermo.rs`): oil
Psat = P_atm·exp((88/R)(1 − Tb/T)), λ = 88·Tb; water Psat on its declared
40 650 J/mol, λ(T) by Kirchhoff on cp_vapour; liquids h = M·cp·(T − 273.15).

Run: python -I wet_column.py   (the dry case; the wet one is check_engine.py's)
"""

import json
import math
import sys

import numpy as np
from scipy.optimize import root

R_GAS = 8.314_462_618_153_24
P_ATM = 101_325.0
T_REF = 273.15
TROUTON = 88.0

WATER = dict(name="water", tb=373.15, m=0.018015, cp=4184.0, dh=40_650.0, cpv=2000.0)


def psat_oil(cut, t):
    return P_ATM * math.exp((TROUTON / R_GAS) * (1.0 - cut["tb"] / t))


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


def psat_water(t):
    low, high = IF97_RANGE
    if t < low:
        end, slope = low, IF97_END_SLOPES[0]
    elif t > high:
        end, slope = high, IF97_END_SLOPES[1]
    else:
        return _if97_psat(t)
    return _if97_psat(end) * math.exp(slope * (1.0 / t - 1.0 / end))



def h_oil(cut, t):
    return cut["m"] * cut["cp"] * (t - T_REF)


def big_h_oil(cut, t):
    return h_oil(cut, t) + TROUTON * cut["tb"]


def h_water(t):
    return WATER["m"] * WATER["cp"] * (t - T_REF)


def big_h_water(t):
    w = WATER
    latent = w["dh"] + (w["cpv"] - w["cp"]) * w["m"] * (t - w["tb"])
    return h_water(t) + latent


def wet_bubble(cuts, y, p, wet):
    """Σ Psat_c·y_c/P (+ Pw/P where water stands) = 1, by bisection."""
    def excess(t):
        s = sum(psat_oil(c, t) * yc for c, yc in zip(cuts, y)) / p
        if wet:
            s += psat_water(t) / p
        return s - 1.0
    lo, hi = 200.0, 900.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if excess(mid) < 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def feed_flash_share(case):
    """The share of the feed's MOLES that is vapour once it has flashed
    adiabatically at the column's pressure — the column's reading (§67.17 rule
    11): the temperature T* where the free-water flash's vapour fraction equals
    the share the feed's sensible heat pays for, cp·(T_F − T*)/λ̄(y(T*)), the
    share the energy's. Bisection from the feed's wet bubble point.

    The free-water flash at (T, P), each cut by Raoult at the total pressure:
    while K_w < 1 water may stand; above it every drop is vapour, and β solves
    Σ_oil z(K−1)/(1+β(K−1)) + z_w/β = 0 on [z_w, 1].
    """
    cuts = case["cuts"]
    p = case["pressure"]
    oil_mass = case["feed_mass"] * (1.0 - case["water_mass_fraction"])
    oil = oil_mass * np.array(case["oil_mass_fractions"]) / np.array([c["m"] for c in cuts])
    water = case["feed_mass"] * case["water_mass_fraction"] / WATER["m"]
    total = oil.sum() + water
    z, zw = oil / total, water / total
    tf = case["feed_temperature"]
    cp = sum(zc * c["m"] * c["cp"] for zc, c in zip(z, cuts)) + zw * WATER["m"] * WATER["cp"]

    def flash(t):
        k = np.array([psat_oil(c, t) / p for c in cuts])
        kw = psat_water(t) / p
        assert kw > 1.0, "the fixture's flash keeps water all vapour"

        def f(beta):
            return float(np.sum(z * (k - 1) / (1 + beta * (k - 1)))) + zw / beta
        if f(1.0) >= 0.0:
            beta = 1.0
        else:
            lo, hi = zw, 1.0
            for _ in range(200):
                mid = 0.5 * (lo + hi)
                if f(mid) > 0.0:
                    lo = mid
                else:
                    hi = mid
            beta = 0.5 * (lo + hi)
        y = z * k / (1 + beta * (k - 1))
        latent = sum(yc * TROUTON * c["tb"] for yc, c in zip(y, cuts))
        latent += (zw / beta) * (WATER["dh"] + (WATER["cpv"] - WATER["cp"]) * WATER["m"] * (t - WATER["tb"]))
        return beta, cp * (tf - t) / latent

    low = wet_bubble(cuts, z / z.sum(), p, True)
    high = tf
    for _ in range(200):
        mid = 0.5 * (low + high)
        flashed, paid = flash(mid)
        if flashed < paid:
            low = mid
        else:
            high = mid
    return flash(0.5 * (low + high))[1]


def solve(case, wet_set, guess=None, newly_wet=frozenset(), blend=1.0):
    cuts = case["cuts"]
    nc = len(cuts)
    n = case["stages"]
    f = case["feed_stage"] - 1
    p = case["pressure"]
    r = case["reflux_ratio"]
    oil_mass = case["feed_mass"] * (1.0 - case["water_mass_fraction"])
    oil_w = np.array(case["oil_mass_fractions"])
    feed_c = oil_mass * oil_w / np.array([c["m"] for c in cuts])  # mol/s
    feed_w = case["feed_mass"] * case["water_mass_fraction"] / WATER["m"]
    # The feed's enthalpy flux on the liquid datum, water included, at its
    # liquid-equivalent temperature (what the engine hands a column).
    tf = case["feed_temperature"]
    feed_h = sum(fc * h_oil(c, tf) for fc, c in zip(feed_c, cuts)) + feed_w * h_water(tf)
    side = case["side_draws"]  # [(stage 1-based, ratio)]
    side_index = {s - 1: k for k, (s, _) in enumerate(side)}
    q_side = np.zeros(n)
    for s, mw in case.get("pumparounds", []):
        q_side[s - 1] = mw * 1e6
    per = 2 * nc + 3
    # The engine's vapour floor (§65.3) where it binds: the vapour leaving
    # `floor = (stage, mol/s)` held there, the reflux ratio the unknown.
    floor = case.get("floor")
    size = n * per + len(side) + (1 if floor else 0)

    def unpack(u):
        st = []
        for j in range(n):
            b = j * per
            st.append(dict(l=u[b:b + nc], v=u[b + nc:b + 2 * nc], t=u[b + 2 * nc],
                           w=u[b + 2 * nc + 1], fw=u[b + 2 * nc + 2]))
        rates = u[n * per:n * per + len(side)]
        return st, rates

    def residual(u):
        st, rates = unpack(u)
        res = []
        r = u[-1] if floor else case["reflux_ratio"]
        v1 = st[0]["v"]
        reflux_c = v1 * r / (r + 1.0)
        d_c = v1 / (r + 1.0)
        y1 = v1 / v1.sum()
        tc = wet_bubble(cuts, y1, p, st[0]["w"] > 0.0)
        for j in range(n):
            s = st[j]
            l_tot = s["l"].sum()
            x = s["l"] / l_tot
            vt = s["v"].sum() + s["w"]
            # liquid from above (oil) and its temperature
            if j == 0:
                l_in, t_in, fw_in = reflux_c, tc, 0.0
            else:
                a = st[j - 1]
                drawn = rates[side_index[j - 1]] if (j - 1) in side_index else 0.0
                share = 1.0 - drawn / a["l"].sum()
                l_in, t_in, fw_in = a["l"] * share, a["t"], a["fw"]
            if j + 1 < n:
                b = st[j + 1]
                v_in, w_in, t_below = b["v"], b["w"], b["t"]
            else:
                v_in, w_in, t_below = np.zeros(nc), 0.0, s["t"]
            fc = feed_c if j == f else np.zeros(nc)
            fw = feed_w if j == f else 0.0
            # M (scaled by the feed's moles)
            scale = feed_c.sum()
            res.extend((l_in + v_in + fc - s["l"] - s["v"]) / scale)
            # E
            k = np.array([psat_oil(c, s["t"]) / p for c in cuts])
            res.extend((s["v"] - k * x * vt) / scale)
            # water
            res.append((w_in + fw_in + fw - s["w"] - s["fw"]) / scale)
            if j in wet_set:
                saturated = (s["w"] * p - psat_water(s["t"]) * vt) / (p * scale)
                if j in newly_wet:
                    # A stage just turned wet slides from "no free water" to
                    # "saturated" — a homotopy for the solve, gone at blend 1.
                    saturated = blend * saturated + (1.0 - blend) * s["fw"] / scale
                res.append(saturated)
            else:
                res.append(s["fw"] / scale)
            # H, all but the reboiler
            if j < n - 1:
                e_in = sum(l * h_oil(c, t_in) for l, c in zip(l_in, cuts)) + fw_in * h_water(t_in)
                e_in += sum(v * big_h_oil(c, t_below) for v, c in zip(v_in, cuts))
                e_in += w_in * big_h_water(t_below)
                e_in += feed_h if j == f else 0.0
                e_out = sum(l * h_oil(c, s["t"]) for l, c in zip(s["l"], cuts))
                e_out += sum(v * big_h_oil(c, s["t"]) for v, c in zip(s["v"], cuts))
                e_out += s["w"] * big_h_water(s["t"]) + s["fw"] * h_water(s["t"]) + q_side[j]
                res.append((e_in - e_out) / (scale * 40_000.0))
        # draw specs
        md = sum(dc * c["m"] for dc, c in zip(d_c, cuts))
        res.append((md - case["distillate_ratio"] * oil_mass) / oil_mass)
        for k, (stage, ratio) in enumerate(side):
            x = st[stage - 1]["l"] / st[stage - 1]["l"].sum()
            mbar = sum(xc * c["m"] for xc, c in zip(x, cuts))
            res.append((rates[k] * mbar - ratio * oil_mass) / oil_mass)
        if floor:
            res.append((st[floor[0] - 1]["v"].sum() - floor[1]) / feed_c.sum())
        return np.array(res)

    if guess is None:
        u = np.zeros(size)
        oil_moles = feed_c.sum()
        z = feed_c / oil_moles
        for j in range(n):
            b = j * per
            frac = j / max(n - 1, 1)
            v = (r + 1.0) * case["distillate_ratio"] * oil_moles
            u[b:b + nc] = z * (r * case["distillate_ratio"] + (1.0 if j >= f else 0.0)) * oil_moles
            u[b + nc:b + 2 * nc] = z * v
            u[b + 2 * nc] = case["guess_top"] + frac * (case["guess_bottom"] - case["guess_top"])
            u[b + 2 * nc + 1] = feed_w if j <= f else 0.0
            u[b + 2 * nc + 2] = 0.0
        for k, (stage, ratio) in enumerate(side):
            u[n * per + k] = ratio * oil_mass / 0.15
    else:
        u = np.array(guess, dtype=float)
        if floor and len(u) < size:
            u = np.append(u, case["reflux_ratio"])
    sol = root(residual, u, method="hybr", options=dict(xtol=1e-13, maxfev=400_000))
    worst = float(np.abs(residual(sol.x)).max())
    if worst > 1e-10:
        # Powell's hybrid stalls where a tray has just turned wet; Levenberg–
        # Marquardt from the same start, then the hybrid again to polish.
        sol = root(residual, u, method="lm", options=dict(xtol=1e-15, maxiter=200_000))
        sol = root(residual, sol.x, method="hybr", options=dict(xtol=1e-13, maxfev=400_000))
        worst = float(np.abs(residual(sol.x)).max())
    return sol.x, worst, unpack, (feed_c, feed_w, feed_h, oil_mass, q_side)


def report(case, wet_set=frozenset(), guess=None, newly_wet=frozenset()):
    for blend in ([0.25, 0.5, 0.75] if newly_wet else []):
        guess, _, _, _ = solve(case, set(wet_set), guess, newly_wet, blend)
    u, worst, unpack, (feed_c, feed_w, feed_h, oil_mass, q_side) = solve(case, set(wet_set), guess)
    st, rates = unpack(u)
    cuts = case["cuts"]
    p = case["pressure"]
    r = u[-1] if case.get("floor") else case["reflux_ratio"]
    n = case["stages"]
    v1 = st[0]["v"]
    d_c = v1 / (r + 1.0)
    y1 = v1 / v1.sum()
    tc = wet_bubble(cuts, y1, p, st[0]["w"] > 0.0)
    # condenser duty: overhead vapour at T1 → oil liquid and free water at Tc
    t1 = st[0]["t"]
    q_cond = sum(v * big_h_oil(c, t1) for v, c in zip(v1, cuts)) + st[0]["w"] * big_h_water(t1)
    q_cond -= sum(v * h_oil(c, tc) for v, c in zip(v1, cuts)) + st[0]["w"] * h_water(tc)
    # reboiler duty: the reboiler stage's own balance
    j = n - 1
    a = st[j - 1]
    drawn = 0.0
    for k, (stage, _) in enumerate(case["side_draws"]):
        if stage - 1 == j - 1:
            drawn = rates[k]
    share = 1.0 - drawn / a["l"].sum()
    e_in = sum(l * share * h_oil(c, a["t"]) for l, c in zip(a["l"], cuts)) + a["fw"] * h_water(a["t"])
    e_in += feed_h if j == case["feed_stage"] - 1 else 0.0
    s = st[j]
    e_out = sum(l * h_oil(c, s["t"]) for l, c in zip(s["l"], cuts))
    e_out += sum(v * big_h_oil(c, s["t"]) for v, c in zip(s["v"], cuts))
    e_out += s["w"] * big_h_water(s["t"]) + s["fw"] * h_water(s["t"])
    q_reb = e_out - e_in
    saturation = []
    for jj in range(n):
        vt = st[jj]["v"].sum() + st[jj]["w"]
        saturation.append((st[jj]["w"] / vt) / (psat_water(st[jj]["t"]) / p))
    bottoms = st[n - 1]["l"]
    bottoms_mass = sum(b * c["m"] for b, c in zip(bottoms, cuts))

    def mass_fractions(moles, water=0.0):
        masses = [m * c["m"] for m, c in zip(moles, cuts)] + [water * WATER["m"]]
        total = sum(masses)
        return [m / total for m in masses]

    out = dict(
        case=case["name"],
        wet_set=sorted(int(x) + 1 for x in wet_set),
        worst_residual=worst,
        temperatures_c=[round(s["t"] - 273.15, 6) for s in st],
        water_vapour_mol_s=[s["w"] for s in st],
        free_water_mol_s=[s["fw"] for s in st],
        saturation=saturation,
        condenser_c=tc - 273.15,
        water_draw_kg_s=st[0]["w"] * WATER["m"],
        bottoms_water_kg_s=st[n - 1]["fw"] * WATER["m"],
        distillate_kg_s=sum(dc * c["m"] for dc, c in zip(d_c, cuts)),
        distillate_mass_fractions=mass_fractions(d_c),
        bottoms_kg_s=bottoms_mass + st[n - 1]["fw"] * WATER["m"],
        bottoms_mass_fractions=mass_fractions(bottoms, st[n - 1]["fw"]),
        condenser_duty_w=q_cond,
        reboiler_duty_w=q_reb,
        top_oil_vapour_mol_s=float(v1.sum()),
        reflux_ratio=float(r),
        feed_vapour_fraction=feed_flash_share(case),
        liquid_mol_s=[float(s["l"].sum()) for s in st],
        oil_vapour_mol_s=[float(s["v"].sum()) for s in st],
        # The engine's vapour floor (heat_balance.rs, VAPOUR_FLOOR_OF_FEED):
        # 20 % of the oil's moles on every stage below the top. A case whose
        # vapour falls under it is one the engine answers at a higher reflux.
        vapour_floor_mol_s=0.2 * float(feed_c.sum()),
    )
    return out, u


# Case A — a wet naphtha-and-kerosene feed whose water rides up as vapour: every
# tray hotter than water's dew point at its own share of the vapour.
DRY_TRAYS = dict(
    name="dry_trays",
    cuts=[
        dict(name="light", tb=343.15, m=0.095, cp=2200.0),
        dict(name="naphtha", tb=423.15, m=0.13, cp=2100.0),
        dict(name="kerosene", tb=493.15, m=0.17, cp=2050.0),
        dict(name="diesel", tb=573.15, m=0.22, cp=2000.0),
    ],
    oil_mass_fractions=[0.2, 0.3, 0.3, 0.2],
    water_mass_fraction=0.01,
    feed_mass=10.0,
    feed_temperature=273.15 + 230.0,
    pressure=1.8e5,
    stages=8,
    feed_stage=5,
    reflux_ratio=2.0,
    distillate_ratio=0.3,
    side_draws=[(3, 0.25)],
    guess_top=273.15 + 110.0,
    guess_bottom=273.15 + 260.0,
)

# Case B — a lighter top product, more water and less reflux at 1 atm: dry, the
# top tray's vapour would pass water's saturation (23 % over), so free water
# stands on it and runs down to re-boil on the hotter tray below. At reflux 1
# its stripping vapour falls under the engine's floor, which the engine meets
# by raising the reflux: `check_engine.py` solves that case, from the engine's
# own state.
WET_TRAYS = dict(DRY_TRAYS)
WET_TRAYS.update(
    name="wet_trays",
    oil_mass_fractions=[0.3, 0.3, 0.2, 0.2],
    water_mass_fraction=0.04,
    pressure=1.01325e5,
    reflux_ratio=1.0,
    distillate_ratio=0.25,
    feed_temperature=273.15 + 200.0,
    side_draws=[(6, 0.15)],
    guess_top=273.15 + 80.0,
    guess_bottom=273.15 + 240.0,
)

# A case's solve starts from the same column at 1 % water, reflux 2 and a 200 °C
# feed, which
# converges from the flat guess, walked to the case in `WALK_STEPS` steps — the
# trays' wet/dry set settled at every step.
WALK_STEPS = 8


def settle(case, wet, guess):
    """Dry everywhere (or `wet`), then flip: a dry stage past saturation turns
    wet, a wet stage whose free water would be negative turns dry."""
    newly_wet = set()
    for _ in range(case["stages"] + 1):
        out, guess = report(case, wet, guess, frozenset(newly_wet))
        if out["worst_residual"] > 1e-10:
            raise RuntimeError(f"{case['name']}: no solve (residual {out['worst_residual']:.2e})")
        over = [j for j, s in enumerate(out["saturation"]) if j not in wet and s > 1.0 + 1e-9]
        negative = [j for j in wet if out["free_water_mol_s"][j] < 0.0]
        if not over and not negative:
            if min(out["liquid_mol_s"]) <= 0.0:
                raise RuntimeError(f"{case['name']}: a tray runs dry {out['liquid_mol_s']}")
            if min(out["oil_vapour_mol_s"][1:]) < out["vapour_floor_mol_s"]:
                print(f"  {case['name']}: under the engine's vapour floor "
                      f"{out['vapour_floor_mol_s']:.2f}: {out['oil_vapour_mol_s']}", file=sys.stderr)
            return out, guess
        # Every stage that flips, flips together (the engine's rule too: one at
        # a time walks through sets no column stands in).
        if negative:
            wet.difference_update(negative)
            newly_wet = set()
        else:
            wet.update(over)
            newly_wet = set(over)
    raise RuntimeError("active set did not settle")


def active_set(case):
    wet = set()
    guess = None
    w0, r0, t0 = 0.01, 2.0, 273.15 + 200.0
    for step in range(WALK_STEPS + 1):
        a = step / WALK_STEPS
        here = dict(
            case,
            water_mass_fraction=w0 + a * (case["water_mass_fraction"] - w0),
            reflux_ratio=r0 + a * (case["reflux_ratio"] - r0),
            feed_temperature=t0 + a * (case["feed_temperature"] - t0),
        )
        out, guess = settle(here, wet, guess)
        print(f"  walk {step}: water {here['water_mass_fraction']:.4f}, R {here['reflux_ratio']:.3f}, "
              f"wet {sorted(j + 1 for j in wet)}", file=sys.stderr)
    return out


if __name__ == "__main__":
    # The wet case is `check_engine.py`'s: at its reflux the engine's floor
    # binds, which this walk does not model.
    cases = [DRY_TRAYS]
    if len(sys.argv) > 1:
        cases = [c for c in cases if c["name"] in sys.argv[1:]]
    for case in cases:
        print(json.dumps(active_set(case), indent=1))
