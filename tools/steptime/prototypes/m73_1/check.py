"""M73.1 formula check: the corrected sieve-tray set against published worked examples."""
import math


def csb_lm(ts_mm, flv):
    # Perry 8th ed. eq. 14-90 (Lygeros & Magoulas 1986), m/s, TS mm
    return 0.0105 + 8.127e-4 * ts_mm ** 0.755 * math.exp(-1.463 * flv ** 0.842)


def k2(h_mm):
    # Ogboja & Kuye (1990) fit of Sinnott Fig. 11.30 (Souza et al. 2022 eq. 67)
    return 23.48 + 1.66 * math.log(h_mm)


def how(l_kg_s, rho_l, lw):
    return 750 * (l_kg_s / (rho_l * lw)) ** (2 / 3)


def chord_share(ad_share):
    """lw/D for a segmental downcomer of area share ad_share of the circle."""
    lo, hi = 0.0, math.pi
    for _ in range(80):
        th = 0.5 * (lo + hi)
        if (th - math.sin(th)) / (2 * math.pi) < ad_share:
            lo = th
        else:
            hi = th
    return math.sin(0.5 * lo)


print("chord: Ad 0.10 ->", round(chord_share(0.10), 4), " Ad 0.12 ->", round(chord_share(0.12), 4))

# --- Flooding fit vs chart readings
print("\nFlooding fit vs published chart readings")
for name, ts, flv, read in (("Perry Ex.9 (500 mm, FLG 0.021)", 500, 0.021, 0.095),
                            ("Dutta Ex.5.1 (18 in, FLV 0.0346)", 457.2, 0.0346, 0.285 * 0.3048),
                            ("NPTEL top (600 mm, FLV 0.033)", 600, 0.033, 0.12),
                            ("NPTEL bottom (600 mm, FLV 0.016)", 600, 0.016, 0.11)):
    c = csb_lm(ts, flv)
    print(f"  {name:36s} fit {c:.4f}  chart {read:.4f}  fit/chart {c / read:.3f}")

# Perry Ex.9 % of flood
c = csb_lm(500, 0.021) * (25 / 20) ** 0.2
unf = c * math.sqrt((841 - 0.481) / 0.481)
un = 25500 / 3600 / 0.481 / 4.66
print(f"  Perry Ex.9: C_corr {c:.4f} (Perry 0.0993), % flood {100 * un / unf:.1f} (Perry 'about 80', 0.0756/0.0993 = 76.1)")

# --- NPTEL Lecture 4-5 (Sinnott 3rd ed. method), acetone-water
print("\nNPTEL worked example (top | bottom)")
lw, hw, dh = 1.752, 40.0, 12.0
ah = 0.489
for sec, lmax, rho_l, rho_v, qv, want in (
        ("top", 9.02, 744.0, 2.08, 6.98, dict(how=27.2, howmin=21.5, k2=30.3, uhmin=12.6, uh=14.3, hd=37.5, hr=16.8, ht=122, hdc=8.8, hb=198)),
        ("bottom", 6.571, 955.0, 0.693, 15.84, dict(how=18.7, howmin=14.7, k2=30.2, uhmin=21.8, uh=32.4, hd=50.2, hr=13.0, ht=122, hdc=2.9, hb=184))):
    c0 = 0.88  # NPTEL: Ah/Ap 15 %, t/dh 1 (Sinnott Fig 11.34)
    h1 = how(lmax, rho_l, lw)
    h2 = how(0.7 * lmax, rho_l, lw)
    kk = k2(hw + h2)
    uhmin = (kk - 0.90 * (25.4 - dh)) / math.sqrt(rho_v)
    uh = qv / ah
    hd = 51 * (uh / c0) ** 2 * rho_v / rho_l
    hr = 12.5e3 / rho_l
    ht = hd + hw + h1 + hr
    am = (hw - 10) / 1000 * lw
    hdc = 166 * (lmax / (rho_l * am)) ** 2
    hb = hw + h1 + ht + hdc
    got = dict(how=h1, howmin=h2, k2=kk, uhmin=uhmin, uh=uh, hd=hd, hr=hr, ht=ht, hdc=hdc, hb=hb)
    print(f"  {sec}: " + "  ".join(f"{k} {got[k]:.2f}/{want[k]}" for k in want))
    print(f"        ({' '.join(f'{k}:{got[k] / want[k] - 1:+.1%}' for k in want)})")

# K2 vs chart readings
print("\nK2 fit vs readings: ", [(h, round(k2(h), 2), r) for h, r in ((54.7, 30.2), (61.5, 30.3))])
