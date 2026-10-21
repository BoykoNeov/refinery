"""The loop plant's routes (M72.6, DESIGN §77.15): one jump to every setting
on the screen's grid, and row walks reflux-first and loops-first.

The keys: the furnace (MW), the reflux, the loops' flow (a share of the file's
85 / 95 / 65 kg/s, all three together) and their return temperatures (moved
together from the file's 150 / 215 / 270 °C, in K).

usage: specs_loops.py jumps|walks [furnace,furnace,...]
"""
import sys

FURNACE = [5.0 * i for i in range(9)]           # 0..40 MW
REFLUX = [0.5 * i for i in range(1, 31)]        # 0.5..15
FLOW = [0.25 * i for i in range(9)]             # 0..2
RET = [-60.0 + 20.0 * i for i in range(7)]      # -60..+60 K
R0, F0, T0 = 4.0, 1.0, 0.0                      # the file's


def steps(start, end, step):
    out, x = [], start
    while abs(x - end) > 1e-9:
        x = round(x + (step if end > x else -step), 6)
        out.append(x)
    return out


def g(x):
    return ('%g' % x)


kind = sys.argv[1]
furnaces = [float(f) for f in sys.argv[2].split(',')] if len(sys.argv) > 2 else FURNACE
specs = []
if kind == 'jumps':
    for f in furnaces:
        for r in REFLUX:
            for p in FLOW:
                for t in RET:
                    specs.append('furnace=%s+reflux=%s+flow=%s+ret=%s' % (g(f), g(r), g(p), g(t)))
elif kind == 'walks':
    for f in furnaces:
        head = 'furnace=%s@3000' % g(f)
        # Reflux first: the reflux to R, the flow to P, then the return
        # temperatures down to -60 and up to +60.
        for r in REFLUX:
            rs = ['reflux=%s@300' % g(x) for x in steps(R0, r, 0.5)]
            for p in FLOW:
                ps = ['flow=%s@300' % g(x) for x in steps(F0, p, 0.25)]
                for end in (RET[0], RET[-1]):
                    ts = ['ret=%s@300' % g(x) for x in steps(T0, end, 20.0)]
                    specs.append('>'.join([head] + rs + ps + ts))
        # Loops first: the flow to P, the return temperatures to T, then the
        # reflux down to 0.5 and up to 15.
        for p in FLOW:
            ps = ['flow=%s@300' % g(x) for x in steps(F0, p, 0.25)]
            for t in RET:
                ts = ['ret=%s@300' % g(x) for x in steps(T0, t, 20.0)]
                for end in (REFLUX[0], REFLUX[-1]):
                    rs = ['reflux=%s@300' % g(x) for x in steps(R0, end, 0.5)]
                    specs.append('>'.join([head] + ps + ts + rs))
print(';'.join(specs))
