"""Build a plant's measured key region (M67.3, DESIGN §72.8) from the driver's
RESULT lines: a setting is ALLOWED when its one jump ran and no walk failed on
reaching it (a walk that failed earlier says nothing of the settings it never
reached). Then only the settings reachable from the file's by single screen
steps through allowed ones are kept.

usage: region.py <plant> <jumps.out> <walks.out> <out.json>
"""
import itertools
import os
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
    COIL = None
elif plant == 'stripped':
    KEYS = ['furnace', 'steam', 'pa_mw']
    GRID = {
        'furnace': [37.5 + 2.5 * i for i in range(12)],
        'steam': [round(0.05 * i, 2) for i in range(1, 21)],
        'pa_mw': [float(i) for i in range(13)],
    }
    # The file's furnace (58 MW) is off the grid: its walks start from each
    # grid furnace's own settled file (`file:F:C|`), steam and pumparounds the
    # file's.
    FILE = {'furnace': 58.0, 'steam': 0.4, 'pa_mw': 0.0}
    # Each furnace's settled coil [°C], measured (`str_coils.txt`, beside this).
    coils = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'str_coils.txt')
    COIL = dict(kv.split('=') for kv in open(coils).read().strip().split(','))
else:
    raise SystemExit('plant ' + plant)

ALIASES = {'pa': 'pa_mw'} if plant == 'stripped' else {}


def index(key, value):
    for i, x in enumerate(GRID[key]):
        if abs(x - value) < 1e-6:
            return i
    raise KeyError((key, value))


def setting(values):
    return tuple(index(k, values[k]) for k in KEYS)


def start_of(spec):
    """A spec's starting values and its moves: `file:F:C|…` starts at F."""
    values = dict(FILE)
    if spec.startswith('file:'):
        head, spec = spec.split('|', 1)
        values['furnace'] = float(head.split(':')[1])
    return values, spec


def apply(values, body):
    for kv in body.split('+'):
        k, v = kv.split('=')
        values[ALIASES.get(k, k)] = float(v)


def parse(path):
    for line in open(path, encoding='utf8', errors='replace'):
        m = re.match(r'RESULT (\S+): (RUNS|FAIL|COMMAND REFUSED)(.*)', line)
        if m:
            yield m.group(1), m.group(2), m.group(3)


shape = [len(GRID[k]) for k in KEYS]
jump = {}
for spec, verdict, rest in parse(jumps_out):
    values, moves = start_of(spec)
    apply(values, moves)
    jump[setting(values)] = verdict == 'RUNS'

failed_by_walk = {}
walks = 0
walk_failures = 0
for spec, verdict, rest in parse(walks_out):
    walks += 1
    values, moves = start_of(spec)
    visited = []
    for move in moves.split('>'):
        body = move.split('@')[0]
        apply(values, body)
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

# Reachable from the file's setting by single steps through allowed settings;
# a file setting between grid points starts from both of its neighbours, and
# both must be allowed.
def around(key, value):
    grid = GRID[key]
    for i, x in enumerate(grid):
        if abs(x - value) < 1e-6:
            return [i]
    for i in range(len(grid) - 1):
        if grid[i] < value < grid[i + 1]:
            return [i, i + 1]
    raise KeyError((key, value))


starts = list(itertools.product(*[around(k, FILE[k]) for k in KEYS]))
assert all(allowed[s] for s in starts), starts
seen = set(starts)
frontier = list(starts)
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

region = {
    'keys': KEYS,
    'grid': {k: GRID[k] for k in KEYS},
    'allowed': rows,
}
if COIL:
    # Each grid furnace's settled coil [°C]: the scene judges a press at the
    # furnace whose settled coil the coil stands at now, too (§72.9).
    region['coil_c'] = [float(COIL['%g' % f]) for f in GRID['furnace']]
json.dump(region, open(out_path, 'w'), indent=0)

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
