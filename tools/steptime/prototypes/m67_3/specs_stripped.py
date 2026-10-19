"""The stripped crude unit's routes (M67.4, DESIGN §72.9). Every run starts
from the plant file with its furnace at F and its coil declared at the
temperature it settles at under F (`str_settle`'s measurement, the coil's own
and not the column's), so no run waits on the coil's drift. From there, after
20 ticks:
- one jump: the steam valve and the pumparounds at once, then 100 ticks;
- steam first: the valve in screen steps from the file's 40 % to S, then the
  pumparounds up from 0 to 12 MW, a press every 20 ticks;
- pumparounds first: the pumparounds up from 0 to P, then the valve down to
  5 % and up to 100 %, a press every 20 ticks.

usage: specs_stripped.py jumps|walks <coil map: F=C,F=C,...>
"""
import sys

FURNACE = [37.5 + 2.5 * i for i in range(12)]    # 37.5..65 MW (30-35: the start fails, C18)
STEAM = [round(0.05 * i, 2) for i in range(1, 21)]  # 5..100 %
PA = [float(i) for i in range(13)]               # 0..12 MW each
S0, P0 = 0.4, 0.0                                # the file's


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
            moves += ['pa=%s@20' % g(x) for x in steps(P0, 12.0, 1.0)]
            specs.append(head + '>'.join(moves))
        for p in PA:
            ps = ['pa=%s@20' % g(x) for x in steps(P0, p, 1.0)]
            for end in (0.05, 1.0):
                ss = ['steam=%s@20' % g(x) for x in steps(S0, end, 0.05)]
                if ps or ss:
                    specs.append(head + '>'.join(ps + ss))
print(';'.join(specs))
