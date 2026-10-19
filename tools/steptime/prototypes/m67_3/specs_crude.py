"""The crude unit's routes (M67.3, DESIGN §72.8): one jump to every setting
on the screen's grid, and row walks reflux-first and pumparounds-first."""
import sys

FURNACE = [2.5 * i for i in range(17)]          # 0..40 MW
REFLUX = [0.5 * i for i in range(1, 31)]        # 0.5..15
PA = [0.25 * i for i in range(17)]              # 0..4 (shares of 15/15/10 MW)
R0, P0 = 4.0, 1.0                               # the file's


def steps(start, end, step):
    out, x = [], start
    while abs(x - end) > 1e-9:
        x = round(x + (step if end > x else -step), 6)
        out.append(x)
    return out


def g(x):
    return ('%g' % x)


kind = sys.argv[1]
specs = []
if kind == 'jumps':
    for f in FURNACE:
        for r in REFLUX:
            for p in PA:
                specs.append('furnace=%s+reflux=%s+pa=%s' % (g(f), g(r), g(p)))
elif kind == 'walks':
    for f in FURNACE:
        head = 'furnace=%s@3000' % g(f)
        # Reflux first: reflux to R, then the pumparounds down to 0 and up to 4.
        for r in REFLUX:
            rs = ['reflux=%s@300' % g(x) for x in steps(R0, r, 0.5)]
            for end in (0.0, 4.0):
                ps = ['pa=%s@300' % g(x) for x in steps(P0, end, 0.25)]
                if not rs and not ps:
                    continue
                specs.append('>'.join([head] + rs + ps))
        # Pumparounds first: the pumparounds to P, then reflux down to 0.5 and up to 15.
        for p in PA:
            ps = ['pa=%s@300' % g(x) for x in steps(P0, p, 0.25)]
            for end in (0.5, 15.0):
                rs = ['reflux=%s@300' % g(x) for x in steps(R0, end, 0.5)]
                specs.append('>'.join([head] + ps + rs))
print(';'.join(specs))
