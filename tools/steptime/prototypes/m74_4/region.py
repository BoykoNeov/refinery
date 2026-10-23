"""M74.4's region builder: M73.4's (below) on the holdup plant, five keys —
the tray plant's four and the reboiler (`reb`: 0 free, else held at that
many MW), on the coarse grid the user chose (DESIGN §79.8); jumps only, no
walks (pass `-` for the walks). A setting is allowed where its jump ran;
a drum or base at a physical limit still runs (the user's decision: shown,
not refused). From M73.4's:

M72.6's region builder (`../m72_6/region.py`) with the tray plant (M73.4,
DESIGN §78.11), whose keys and grid are the loop plant's. From M67.3's
(`../m67_3/region.py`) with the loop plant's four keys (M72.6, §77.15):
`allowed` nests one list per key but the last, whose settings are a string of
"0"/"1" — on three keys M67.3's layout.

Build a plant's measured key region (M67.3, DESIGN §72.8) from the driver's
RESULT lines: a setting is ALLOWED when its one jump ran and no walk failed on
reaching it (a walk that failed earlier says nothing of the settings it never
reached). Then only the settings reachable from the file's by single screen
steps through allowed ones are kept.

usage: region.py <plant> <jumps.out> <walks.out> <out.json>
(each .out may be several files joined by commas: one per batch)
"""
import itertools
import os
import json
import re
import sys

plant, jumps_out, walks_out, out_path = sys.argv[1:5]

if plant == 'holdup':
    KEYS = ['furnace', 'reflux', 'flow', 'ret', 'reb']
    GRID = {
        'furnace': [0.0, 20.0, 40.0],
        'reflux': [0.5, 1.0, 2.0, 4.0, 8.0, 15.0],
        'flow': [0.0, 1.0, 2.0],
        'ret': [-60.0, 0.0, 60.0],
        'reb': [0.0, 60.0, 90.0, 120.0],
    }
    # The file's reboiler held at its first tick's duty, between 60 and 90.
    FILE = {'furnace': 20.0, 'reflux': 4.0, 'flow': 1.0, 'ret': 0.0, 'reb': 86.69}
    COIL = None
elif plant in ('loops', 'trays'):
    KEYS = ['furnace', 'reflux', 'flow', 'ret']
    GRID = {
        'furnace': [5.0 * i for i in range(9)],
        'reflux': [0.5 * i for i in range(1, 31)],
        'flow': [0.25 * i for i in range(9)],
        'ret': [-60.0 + 20.0 * i for i in range(7)],
    }
    FILE = {'furnace': 20.0, 'reflux': 4.0, 'flow': 1.0, 'ret': 0.0}
    COIL = None
elif plant == 'crude':
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
elif plant == 'cold':
    # Its reboiler free (M67.5): the stripped plant's keys but the pumparounds
    # a share of the file's 15/15/10 MW, the furnace from the file's 20 MW.
    KEYS = ['furnace', 'steam', 'pa']
    GRID = {
        'furnace': [20 + 2.5 * i for i in range(19)],
        'steam': [round(0.05 * i, 2) for i in range(1, 21)],
        'pa': [0.25 * i for i in range(5)],
    }
    FILE = {'furnace': 20.0, 'steam': 0.4, 'pa': 1.0}
    COIL = dict(kv.split('=') for kv in open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cold_coils.txt')).read().strip().split(','))
else:
    raise SystemExit('plant ' + plant)

ALIASES = {'pa': 'pa_mw'} if plant == 'stripped' else {}
if plant == 'holdup':
    ALIASES = {'free': 'reb'}


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
        # `free=1`: the reboiler free, 0 on its grid.
        values[ALIASES.get(k, k)] = 0.0 if k == 'free' else float(v)


def parse(paths):
    if paths == '-':
        return
    for line in (l for path in paths.split(',') for l in open(path, encoding='utf8', errors='replace')):
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

def nest(prefix):
    if len(prefix) == len(shape) - 1:
        return ''.join('1' if prefix + (k,) in seen else '0' for k in range(shape[-1]))
    return [nest(prefix + (i,)) for i in range(shape[len(prefix)])]


rows = nest(())

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
