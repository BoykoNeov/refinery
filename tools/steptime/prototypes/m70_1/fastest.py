"""Fastest of the rounds per tick, then medians over windows; solves and iterations a tick."""
import csv, statistics, sys, glob
out = sys.argv[1]
def load(stem):
    runs = [list(csv.DictReader(open(f))) for f in sorted(glob.glob(f'{out}/{stem}_[0-9].csv'))]
    wall = [min(float(r[i]['wall_us']) for r in runs) / 1e3 for i in range(len(runs[0]))]
    return wall, runs[0], len(runs)
for story, windows in [('step', [('settled 1000-1500', 1000, 1500), ('step +1..60', 1500, 1560), ('step +1..600', 1500, 2100), ('step +600..1800', 2100, 3300)]),
                       ('cold', [('cold 100-700', 100, 700), ('cold 700-2400', 700, 2400)])]:
    for v in ['head', 'new']:
        wall, rows, n = load(f'{story}_{v}')
        for name, a, b in windows:
            w = wall[a:b]; rs = rows[a:b]
            print(f"{v:4} {name:18} median {statistics.median(w):5.2f} ms  over 2.5 ms {sum(x > 2.5 for x in w):4}/{len(w)}  "
                  f"solves {sum(1 + int(r['re_solves']) for r in rs) / len(rs):4.2f}  iterations {sum(int(r['iterations']) for r in rs) / len(rs):5.2f}   (fastest of {n})")
