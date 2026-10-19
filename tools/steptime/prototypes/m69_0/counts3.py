"""Per story phase: passes, flow iterations, flash calls per tick, three runs."""
import re
import sys

KEYS = ['engine: flow iterations', 'flash: density', 'flash: settle', 'resolve: coil cells',
        'solver: forced step', 'solver: forced step refused']


def read(path):
    out, cur = {}, None
    for line in open(path, encoding='utf-8', errors='replace'):
        m = re.match(r'== (\S+) \[(.+?)\] (\d+) ticks: wall mean ([\d.]+) ms; passes/tick mean ([\d.]+)', line)
        if m:
            cur = (m[1], m[2])
            out[cur] = {'ticks': int(m[3]), 'wall': float(m[4]), 'passes': float(m[5])}
            continue
        m = re.match(r'\s+(.+?)\s{2,}([\d.]+)/tick', line)
        if m and cur and m[1] in KEYS:
            out[cur][m[1]] = float(m[2])
    return out


runs = [(name, read(path)) for name, path in zip(sys.argv[1::2], sys.argv[2::2])]
cols = ['passes', 'engine: flow iterations', 'flash: density', 'flash: settle',
        'solver: forced step', 'solver: forced step refused']
short = ['passes', 'flow it', 'density', 'settle', 'forced', 'refused']
for key in runs[0][1]:
    print(f'{key[0]} [{key[1]}] ({runs[0][1][key]["ticks"]} ticks)')
    print('    ' + f'{"run":10}' + ''.join(f'{s:>10}' for s in short) + f'{"work":>10}')
    base = None
    for name, r in runs:
        row = r.get(key, {})
        # The work: every flash call (density + settle), the bulk of the tick.
        work = row.get('flash: density', 0) + row.get('flash: settle', 0)
        base = base or work
        print('    ' + f'{name:10}' + ''.join(f'{row.get(c, 0):10.2f}' for c in cols)
              + f'{100 * work / base:9.1f}%')
