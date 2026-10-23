"""Key 8's table grid (M74.4, the user's choice 2026-10-23: coarse, settled):
one jump to every setting, run 24 000 ticks after it.

usage: specs_holdup.py [furnace,furnace,...]
"""
import sys

FURNACE = [0.0, 20.0, 40.0]
REFLUX = [0.5, 1.0, 2.0, 4.0, 8.0, 15.0]
FLOW = [0.0, 1.0, 2.0]
RET = [-60.0, 0.0, 60.0]
REB = ['free', 60.0, 90.0, 120.0]
g = lambda x: '%g' % x
furnaces = [float(f) for f in sys.argv[1].split(',')] if len(sys.argv) > 1 else FURNACE
specs = []
for f in furnaces:
    for r in REFLUX:
        for p in FLOW:
            for t in RET:
                for b in REB:
                    reb = 'free=1' if b == 'free' else 'reb=%s' % g(b)
                    specs.append('furnace=%s+reflux=%s+flow=%s+ret=%s+%s' % (g(f), g(r), g(p), g(t), reb))
print(';'.join(specs))
