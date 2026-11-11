import sys, csv
def load(n):
    with open(f'W:/temp/claude/m75_3_0/{n}.csv') as f:
        return list(csv.DictReader(f))
for n in sys.argv[1:]:
    rows = load(n)
    p = [float(r['p_bar']) for r in rows]
    v = [float(r['vent_kg_s']) for r in rows]
    dp = [b - a for a, b in zip(p, p[1:])]
    flips = sum(1 for a, b in zip(dp, dp[1:]) if a * b < 0 and abs(a) > 1e-7 and abs(b) > 1e-7)
    dv = [b - a for a, b in zip(v, v[1:])]
    vflips = sum(1 for a, b in zip(dv, dv[1:]) if a * b < 0 and abs(a) > 1e-4 and abs(b) > 1e-4)
    first = next((i for i, x in enumerate(v) if x > 0), None)
    print(f"{n}: ticks {len(p)}, P sign flips {flips}, vent sign flips {vflips}, max |dP| {max(abs(d) for d in dp)*1e5:.0f} Pa/tick, first vent tick {first} (t={rows[first]['t_s'] if first is not None else '-'} s), P end {p[-1]:.4f}, vent end {v[-1]:.2f}")
if len(sys.argv) > 2:
    a, b = load(sys.argv[1]), load(sys.argv[2])
    d = max(abs(float(x['p_bar']) - float(y['p_bar'])) for x, y in zip(a, b))
    print(f"max |P_{sys.argv[1]} - P_{sys.argv[2]}| = {d*1e5:.1f} Pa")
