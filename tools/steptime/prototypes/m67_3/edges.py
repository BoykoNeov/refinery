"""The edge band of a committed region (M69.3, DESIGN §74.6): every allowed
cell with a disallowed neighbour (inside) and every disallowed cell with an
allowed one (outside), one step along any key. Each as a jump from its
furnace's settled file, the driver's `file:F:C|steam=…+pa=…` spec.

usage: edges.py <plant> <region.json> <inside.specs> <outside.specs>
"""
import itertools
import json
import os
import sys

plant, path, inside_path, outside_path = sys.argv[1:5]
region = json.load(open(path))
keys, grid = region['keys'], region['grid']
shape = [len(grid[k]) for k in keys]
cells = region['allowed']
coils = dict(kv.split('=') for kv in open(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '%s_coils.txt' % ('str' if plant == 'stripped' else 'cold'))).read().strip().split(','))
g = lambda x: '%g' % x


def ok(s):
    return cells[s[0]][s[1]][s[2]] == '1'


def neighbours(s):
    for a in range(3):
        for d in (-1, 1):
            t = list(s)
            t[a] += d
            if 0 <= t[a] < shape[a]:
                yield tuple(t)


def spec(s):
    v = {k: grid[k][s[a]] for a, k in enumerate(keys)}
    pa = v.get('pa_mw', v.get('pa'))
    return 'file:%s:%s|steam=%s+pa=%s' % (g(v['furnace']), coils[g(v['furnace'])], g(v['steam']), g(pa))


inside, outside = [], []
for s in itertools.product(*[range(n) for n in shape]):
    if ok(s) and any(not ok(t) for t in neighbours(s)):
        inside.append(spec(s))
    elif not ok(s) and any(ok(t) for t in neighbours(s)):
        outside.append(spec(s))
open(inside_path, 'w').write(';'.join(inside))
open(outside_path, 'w').write(';'.join(outside))
print('%s: inside %d, outside %d' % (plant, len(inside), len(outside)))
