"""Tighten a committed region with longer runs (M67's correction, DESIGN
§72.12): every cell a longer run failed is not allowed; on the cold plant a
(furnace, pumparounds) row whose long run failed at any tested steam setting is
closed at every steam setting. Then only what is reachable from the file's
setting stays, and the allowed cells next to the new edge are listed for the
next round.

usage: tighten.py <plant> <region.json> <out-file>... [--rows <band.out>]
       --next <edge.specs>
"""
import itertools
import json
import os
import re
import sys

plant, path = sys.argv[1], sys.argv[2]
args = sys.argv[3:]
rows_out = args[args.index('--rows') + 1] if '--rows' in args else None
next_specs = args[args.index('--next') + 1]
outs = [a for a in args if a.endswith('.out') and a != rows_out]
region = json.load(open(path))
keys, grid = region['keys'], region['grid']
shape = [len(grid[k]) for k in keys]
cells = [[list(row) for row in plane] for plane in region['allowed']]
alias = {'pa': 'pa_mw'} if plant == 'stripped' else {}
FILE = {'stripped': {'furnace': 58.0, 'steam': 0.4, 'pa_mw': 0.0},
        'cold': {'furnace': 20.0, 'steam': 0.4, 'pa': 1.0}}[plant]


def at(key, value):
    for i, x in enumerate(grid[key]):
        if abs(x - value) < 1e-6:
            return i
    raise KeyError((key, value))


def parse(path):
    for line in open(path, encoding='utf8', errors='replace'):
        m = re.match(r'RESULT file:([0-9.]+):[0-9.]+\|(\S+): (RUNS|FAIL)', line)
        if m:
            values = {'furnace': float(m.group(1))}
            for kv in m.group(2).split('+'):
                k, v = kv.split('=')
                values[alias.get(k, k)] = float(v)
            yield tuple(at(k, values[k]) for k in keys), m.group(3) == 'RUNS'


closed = 0
for out in outs:
    for s, runs in parse(out):
        if not runs and cells[s[0]][s[1]][s[2]] == '1':
            cells[s[0]][s[1]][s[2]] = '0'
            closed += 1
rows_closed = 0
if rows_out:
    for s, runs in parse(rows_out):
        if not runs:
            for j in range(shape[1]):
                if cells[s[0]][j][s[2]] == '1':
                    cells[s[0]][j][s[2]] = '0'
                    rows_closed += 1


def ok(s):
    return cells[s[0]][s[1]][s[2]] == '1'


def around(key, value):
    g = grid[key]
    for i, x in enumerate(g):
        if abs(x - value) < 1e-6:
            return [i]
    return [i for i in range(len(g) - 1) if g[i] < value < g[i + 1]][:1] + \
        [i + 1 for i in range(len(g) - 1) if g[i] < value < g[i + 1]][:1]


starts = list(itertools.product(*[around(k, FILE[k]) for k in keys]))
assert all(ok(s) for s in starts), starts
seen, frontier = set(starts), list(starts)
while frontier:
    s = frontier.pop()
    for axis in range(3):
        for d in (-1, 1):
            t = list(s)
            t[axis] += d
            t = tuple(t)
            if 0 <= t[axis] < shape[axis] and ok(t) and t not in seen:
                seen.add(t)
                frontier.append(t)
everything = list(itertools.product(*[range(n) for n in shape]))
for s in everything:
    if ok(s) and s not in seen:
        cells[s[0]][s[1]][s[2]] = '0'
region['allowed'] = [[''.join(row) for row in plane] for plane in cells]
json.dump(region, open(path, 'w'), indent=0)

edge = []
for s in everything:
    if not ok(s):
        continue
    if any(0 <= s[a] + d < shape[a] and not ok(tuple(s[b] + (d if b == a else 0) for b in range(3)))
           for a in range(3) for d in (-1, 1)):
        edge.append(s)
coils = dict(kv.split('=') for kv in open(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '%s_coils.txt' % ('str' if plant == 'stripped' else 'cold'))).read().strip().split(','))
g = lambda x: '%g' % x
specs = []
for s in edge:
    v = {k: grid[k][s[a]] for a, k in enumerate(keys)}
    pa = v.get('pa_mw', v.get('pa'))
    specs.append('file:%s:%s|steam=%s+pa=%s' % (g(v['furnace']), coils[g(v['furnace'])], g(v['steam']), g(pa)))
open(next_specs, 'w').write(';'.join(specs))
print('%s: closed %d cells, %d by rows; allowed %d; next edge %d' % (
    plant, closed, rows_closed, sum(ok(s) for s in everything), len(edge)))
