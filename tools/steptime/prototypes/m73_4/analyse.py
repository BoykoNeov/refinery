"""M73.4's read of the batches (DESIGN §78.11): what failed and why, whether
the trays moved the loop plant's edge, and where the trays flood and weep.

usage: analyse.py <jumps.out,...> <walks.out,...> <trays region.json> <loops region.json>
"""
import itertools
import json
import re
import sys
from collections import Counter, defaultdict

jumps_out, walks_out, trays_json, loops_json = sys.argv[1:5]
KEYS = ['furnace', 'reflux', 'flow', 'ret']
GRID = {
    'furnace': [5.0 * i for i in range(9)],
    'reflux': [0.5 * i for i in range(1, 31)],
    'flow': [0.25 * i for i in range(9)],
    'ret': [-60.0 + 20.0 * i for i in range(7)],
}


def lines(paths):
    for path in paths.split(','):
        for line in open(path, encoding='utf8', errors='replace'):
            if line.startswith('RESULT '):
                yield line.rstrip('\n')


def reason(rest):
    """A failure's kind, in a few words."""
    for key, words in [
        ('did not converge', 'cascade did not converge'),
        ('no step that improves', 'no step improves'),
        ('solves, each stage at the pressure', 'the pressure re-solve gave up'),
        ('starting point is outside the physical region', 'its start left the physical region'),
        ('COMMAND REFUSED', 'command refused'),
    ]:
        if key in rest:
            return words
    return rest[:120]


def values_of(spec):
    v = {'furnace': 20.0, 'reflux': 4.0, 'flow': 1.0, 'ret': 0.0}
    for move in spec.split('>'):
        for kv in move.split('@')[0].split('+'):
            k, x = kv.split('=')
            v[k] = float(x)
    return v


def g(x):
    return '%g' % x


# --- jumps
jump_runs, jump_fails = 0, 0
fail_reasons = Counter()
fail_after = Counter()
readings = {}
for line in lines(jumps_out):
    m = re.match(r'RESULT (\S+): (RUNS|FAIL)(.*)', line)
    spec, verdict, rest = m.groups()
    v = values_of(spec)
    key = tuple(v[k] for k in KEYS)
    if verdict == 'RUNS':
        jump_runs += 1
        r = re.match(r' flood (\S+) weep (\S+) maxflood (\S+) maxdc (\S+) minweep (\S+) span (\S+) mbar', rest)
        flood, weep, maxflood, maxdc, minweep, span = r.groups()
        readings[key] = {
            'flood': [] if flood == '-' else [int(x) for x in flood.split(',')],
            'weep': [] if weep == '-' else [int(x) for x in weep.split(',')],
            'maxflood': float(maxflood), 'maxdc': float(maxdc), 'minweep': float(minweep), 'span': float(span),
        }
    else:
        jump_fails += 1
        fail_reasons[reason(rest)] += 1
        a = re.search(r'\((-?\d+) after the last move', rest)
        fail_after[int(a.group(1)) if a else None] += 1
print('JUMPS: %d ran, %d failed' % (jump_runs, jump_fails))
print('  failure kinds:', dict(fail_reasons))
print('  ticks after the jump at failure:', dict(sorted(fail_after.items(), key=lambda kv: (kv[0] is None, kv[0]))))

# --- walks
walk_runs, walk_fails = 0, 0
walk_reasons = Counter()
for line in lines(walks_out):
    m = re.match(r'RESULT (\S+): (RUNS|FAIL|COMMAND REFUSED)(.*)', line)
    if m.group(2) == 'RUNS':
        walk_runs += 1
    else:
        walk_fails += 1
        walk_reasons[reason(m.group(3))] += 1
print('WALKS: %d ran, %d failed' % (walk_runs, walk_fails))
print('  failure kinds:', dict(walk_reasons))

# --- the trays' region against the loop plant's
def allowed_set(path):
    r = json.load(open(path))
    out = set()
    shape = [len(r['grid'][k]) for k in r['keys']]
    for s in itertools.product(*[range(n) for n in shape]):
        node = r['allowed']
        for i in s[:-1]:
            node = node[i]
        if node[s[-1]] == '1':
            out.add(tuple(r['grid'][k][s[a]] for a, k in enumerate(r['keys'])))
    return out


trays = allowed_set(trays_json)
loops = allowed_set(loops_json)
print('REGION: trays allow %d, loops allow %d; both %d; trays only %d; loops only %d' % (
    len(trays), len(loops), len(trays & loops), len(trays - loops), len(loops - trays)))
for name, diff in (('allowed on the trays only', trays - loops), ('allowed on the loops only', loops - trays)):
    by_furnace = defaultdict(list)
    for s in sorted(diff):
        by_furnace[s[0]].append(s[1:])
    print('  %s, by furnace:' % name)
    for f in sorted(by_furnace):
        rows = by_furnace[f]
        print('    %g MW: %d — %s%s' % (f, len(rows), ' '.join('r%s/%s%%/%+gK' % (g(r), g(p * 100), t) for r, p, t in rows[:12]),
                                       ' …' if len(rows) > 12 else ''))

# --- the edge, as M72.6's table: the least flow allowed per (reflux, return)
print('EDGE (least loop flow allowed), per furnace group:')
for group in ([0.0, 5.0, 10.0, 15.0, 20.0], [25.0], [30.0], [35.0], [40.0]):
    tables = []
    for f in group:
        t = {}
        for r in GRID['reflux']:
            for ret in GRID['ret']:
                ok = [p for p in GRID['flow'] if (f, r, p, ret) in trays]
                t[(r, ret)] = min(ok) if ok else None
        tables.append(t)
    same = all(t == tables[0] for t in tables)
    print('  furnace %s MW%s' % ('/'.join(g(f) for f in group), '' if same else ' (NOT the same at each)'))
    t = tables[0]
    print('    reflux ' + ' '.join('%7s' % ('%+gK' % x) for x in GRID['ret']))
    for r in GRID['reflux']:
        row = [t[(r, ret)] for ret in GRID['ret']]
        if all(x == 0.0 for x in row):
            continue
        print('    %5s  ' % g(r) + ' '.join('%7s' % ('none' if x is None else '%g%%' % (x * 100)) for x in row))

# --- the trays: where they flood and weep, on the allowed settings
print('TRAYS at the file\'s loops (100 %, +0 K), along the reflux:')
for f in (0.0, 20.0, 40.0):
    print('  furnace %g MW' % f)
    for r in GRID['reflux']:
        k = (f, r, 1.0, 0.0)
        if k not in readings:
            print('    reflux %5s: FAILED' % g(r))
            continue
        d = readings[k]
        print('    reflux %5s: flood %-28s weep %-20s maxflood %.2f maxdc %.2f minweep %.2f span %.0f mbar' % (
            g(r), ','.join(map(str, d['flood'])) or '-', ','.join(map(str, d['weep'])) or '-',
            d['maxflood'], d['maxdc'], d['minweep'], d['span']))
print('TRAYS over the allowed settings, per reflux: share with a tray flooding / weeping')
for r in GRID['reflux']:
    ks = [k for k in readings if k[1] == r and k in trays]
    if not ks:
        continue
    fl = sum(1 for k in ks if readings[k]['flood'])
    we = sum(1 for k in ks if readings[k]['weep'])
    span = max(readings[k]['span'] for k in ks)
    print('  reflux %5s: %4d settings, flooding %5.1f %%, weeping %5.1f %%, span up to %.0f mbar' % (
        g(r), len(ks), 100.0 * fl / len(ks), 100.0 * we / len(ks), span))
inside = [k for k in readings if k in trays and not readings[k]['flood'] and not readings[k]['weep']]
print('allowed settings with every tray inside its window: %d of %d' % (len(inside), len(trays)))
