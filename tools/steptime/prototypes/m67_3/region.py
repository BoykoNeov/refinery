"""Build a plant's measured key region (M67.3, DESIGN §72.8) from the driver's
RESULT lines: a setting is ALLOWED when its one jump ran and no walk failed on
reaching it (a walk that failed earlier says nothing of the settings it never
reached). Then only the settings reachable from the file's by single screen
steps through allowed ones are kept.

usage: region.py <plant> <jumps.out> <walks.out> <out.json>
"""
import itertools
import json
import re
import sys

plant, jumps_out, walks_out, out_path = sys.argv[1:5]

if plant == 'crude':
    KEYS = ['furnace', 'reflux', 'pa']
    GRID = {
        'furnace': [2.5 * i for i in range(17)],
        'reflux': [0.5 * i for i in range(1, 31)],
        'pa': [0.25 * i for i in range(17)],
    }
    FILE = {'furnace': 20.0, 'reflux': 4.0, 'pa': 1.0}
else:
    raise SystemExit('plant ' + plant)


def index(key, value):
    for i, x in enumerate(GRID[key]):
        if abs(x - value) < 1e-6:
            return i
    raise KeyError((key, value))


def setting(values):
    return tuple(index(k, values[k]) for k in KEYS)


def parse(path):
    for line in open(path, encoding='utf8', errors='replace'):
        m = re.match(r'RESULT (\S+): (RUNS|FAIL|COMMAND REFUSED)(.*)', line)
        if m:
            yield m.group(1), m.group(2), m.group(3)


shape = [len(GRID[k]) for k in KEYS]
jump = {}
for spec, verdict, rest in parse(jumps_out):
    values = {k: float(v) for k, v in (kv.split('=') for kv in spec.split('+'))}
    jump[setting(values)] = verdict == 'RUNS'

failed_by_walk = {}
walks = 0
walk_failures = 0
for spec, verdict, rest in parse(walks_out):
    walks += 1
    values = dict(FILE)
    visited = []
    for move in spec.split('>'):
        body = move.split('@')[0]
        for kv in body.split('+'):
            k, v = kv.split('=')
            values[k] = float(v)
        visited.append((body, setting(values)))
    if verdict == 'RUNS':
        continue
    walk_failures += 1
    m = re.search(r'last sent ([^)]*)\)', rest)
    last = m.group(1)
    for body, s in visited:
        if body == last:
            failed_by_walk.setdefault(s, []).append(spec)
            break
    else:
        raise SystemExit('no move %s in %s' % (last, spec))

missing = [s for s in itertools.product(*[range(n) for n in shape]) if s not in jump]
assert not missing, missing[:5]
allowed = {s: jump[s] and s not in failed_by_walk for s in jump}

# Reachable from the file's setting by single steps through allowed settings.
start = setting(FILE)
assert allowed[start]
seen = {start}
frontier = [start]
while frontier:
    s = frontier.pop()
    for axis in range(len(KEYS)):
        for d in (-1, 1):
            t = list(s)
            t[axis] += d
            t = tuple(t)
            if 0 <= t[axis] < shape[axis] and allowed[t] and t not in seen:
                seen.add(t)
                frontier.append(t)

rows = []
for i in range(shape[0]):
    plane = []
    for j in range(shape[1]):
        plane.append(''.join('1' if (i, j, k) in seen else '0' for k in range(shape[2])))
    rows.append(plane)

json.dump({
    'keys': KEYS,
    'grid': {k: GRID[k] for k in KEYS},
    'allowed': rows,
}, open(out_path, 'w'), indent=0)

n = len(jump)
print('settings %d; jumps run %d; walks %d, failed %d, settings a walk failed on %d; '
      'allowed %d; reachable %d' % (
          n, sum(jump.values()), walks, walk_failures, len(failed_by_walk),
          sum(allowed.values()), len(seen)))
# The settings a walk failed on whose jump ran: the route's own edge.
route_only = sorted(s for s in failed_by_walk if jump[s])
print('route-only failures (jump ran, a walk failed): %d' % len(route_only))
for s in route_only[:20]:
    print('  ', {k: GRID[k][s[a]] for a, k in enumerate(KEYS)})
