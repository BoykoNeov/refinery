"""The cold crude unit's routes with its reboiler free (M67.5, DESIGN §72.10).
As the stripped plant's (`specs_stripped.py`): every run starts from the plant
file with its furnace at F and its coil declared where F settles it, then after
20 ticks:
- one jump: the steam valve and the pumparounds at once, then 100 ticks;
- steam first: the valve in screen steps from the file's 40 % to S, then the
  pumparounds down from the file's 100 % to none, a press every 20 ticks;
- pumparounds first: the pumparounds to P, then the valve down to 5 % and up
  to 100 %, a press every 20 ticks.
The pumparounds are a share of the file's 15/15/10 MW, as the screen's keys
send them.

usage: specs_cold.py jumps|walks <coil map: F=C,F=C,...>
"""
import sys

FURNACE = [20 + 2.5 * i for i in range(19)]       # 20..65 MW
STEAM = [round(0.05 * i, 2) for i in range(1, 21)]  # 5..100 %
PA = [0.25 * i for i in range(5)]                 # 0..100 %
S0, P0 = 0.4, 1.0                                 # the file's


def g(x):
    return '%g' % x


def steps(start, end, step):
    out, x = [], start
    while abs(x - end) > 1e-9:
        x = round(x + (step if end > x else -step), 6)
        out.append(x)
    return out


kind = sys.argv[1]
coil = {float(f): c for f, c in (kv.split('=') for kv in sys.argv[2].split(','))}
specs = []
for f in FURNACE:
    head = 'file:%s:%s|' % (g(f), coil[f])
    if kind == 'jumps':
        for s in STEAM:
            for p in PA:
                specs.append(head + 'steam=%s+pa=%s' % (g(s), g(p)))
    else:
        for s in STEAM:
            moves = ['steam=%s@20' % g(x) for x in steps(S0, s, 0.05)]
            moves += ['pa=%s@20' % g(x) for x in steps(P0, 0.0, 0.25)]
            specs.append(head + '>'.join(moves))
        for p in PA:
            ps = ['pa=%s@20' % g(x) for x in steps(P0, p, 0.25)]
            for end in (0.05, 1.0):
                ss = ['steam=%s@20' % g(x) for x in steps(S0, end, 0.05)]
                if ps or ss:
                    specs.append(head + '>'.join(ps + ss))
print(';'.join(specs))
